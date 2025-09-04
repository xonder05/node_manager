#include "node_manager/node_manager.hpp"

NodeManager::NodeManager() : Node("node_manager")
{
    auto qos = rclcpp::SystemDefaultsQoS()
    .keep_last(10) // history
    .reliable() // reliability
    .transient_local(); // durability

    this->declare_parameter("manager_id", "");
    manager_id = this->get_parameter("manager_id").as_string();

    command_subscriber = this->create_subscription<node_manager::msg::NodeRedCommand>("/management/commands", 10, std::bind(&NodeManager::commandCallback, this, std::placeholders::_1));
    command_publisher = this->create_publisher<node_manager::msg::NodeRedCommand>("/management/commands", qos);

    RCLCPP_INFO(this->get_logger(), "NodeManager initialization done.");
}

NodeManager::~NodeManager() {}

void NodeManager::commandCallback(node_manager::msg::NodeRedCommand::ConstSharedPtr msg) 
{
    if (msg->manager_id != this->manager_id)
    {
        RCLCPP_INFO(this->get_logger(), "Recieved command for someone else, ignoring");
        return;
    }

    switch (msg->message_type)
    {
        case 0:
        {
            int res = CreateNode(msg);
            node_manager::msg::NodeRedCommand reply;
            
            reply.manager_id = this->manager_id;
            reply.message_type = 90;
            reply.node_id = msg->node_id;
            reply.return_value = res;

            this->command_publisher->publish(reply);
            break;
        }
        
        case 1:
        {
            int res = DeleteNode(msg);

            node_manager::msg::NodeRedCommand reply;
            
            reply.manager_id = this->manager_id;
            reply.message_type = 90;
            reply.node_id = msg->node_id;
            reply.return_value = res;

            this->command_publisher->publish(reply);

            break;
        }

        case 2:
        {
            int res = CreateNode(msg);

            node_manager::msg::NodeRedCommand reply;
            
            reply.manager_id = this->manager_id;
            reply.message_type = 90;
            reply.node_id = msg->node_id;
            reply.return_value = res;

            this->command_publisher->publish(reply);

            break;
        }

        default:
            RCLCPP_INFO(this->get_logger(), "I heard unknown message type\n");
            break;
    }
}

std::vector<std::string> NodeManager::ParseJson(std::string json)
{
    boost::json::value json_value;
    try {
        json_value = boost::json::parse(json);
    }
    catch(const boost::json::system_error& e) {
        RCLCPP_WARN(this->get_logger(), "Parameter / Remap is not in json format, ignoring");
        return std::vector<std::string>();
    }

    if(!json_value.is_array())
    {
        RCLCPP_WARN(this->get_logger(), "Parameter / Remap json is not array, ignoring");
        return std::vector<std::string>();
    }

    boost::json::array json_array = json_value.as_array();

    std::vector<std::string> command_parameters;
    for (const auto &item : json_array)
    {
        if(item.is_string()) {
            command_parameters.push_back(std::string(item.as_string()));
        }
    }
    return command_parameters;
}

int NodeManager::ParameterChangeOnNode(std::string node_name, std::vector<std::string> params)
{
    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);

    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Parameter service not available on node %s", node_name);
        return 1;
    }

    for (auto param : params)
    {
        auto pos = param.find(":=");
        if (pos != std::string::npos) 
        {
            std::string name  = param.substr(0, pos);
            std::string value = param.substr(pos + 2);

            auto results = client->set_parameters({rclcpp::Parameter(name, value)});

            for (auto & result : results)
            {
                if (!result.successful) 
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to set param %s: %s", name, result.reason.c_str());
                    return 1;
                }   
            }
        }
    }

    return 0;
}

int NodeManager::CreateNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg)
{
    auto it = nodes.find(msg->node_id);

    // node does not exist yet
    if (it == nodes.end())
    {
        RCLCPP_INFO(this->get_logger(), "Creating a new node");

        node n;
        n.configuration = msg;

        std::vector<std::string> command_params;
        command_params.push_back("run");
        command_params.push_back(n.configuration->package_name);
        command_params.push_back(n.configuration->node_name);
        
        auto node_params = ParseJson(n.configuration->param_json);
        command_params.insert(command_params.end(), node_params.begin(), node_params.end());

        auto node_remaps = ParseJson(n.configuration->remap_json);
        command_params.insert(command_params.end(), node_remaps.begin(), node_remaps.end());

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::String>("/management/stdout/" + n.configuration->node_id, 5);

        n.process = std::make_shared<SubprocessWithRosPublisher>("ros2", command_params, publisher);

        int ret = n.process->Start();
        if (ret < 10) 
        {
            nodes[n.configuration->node_id] = n;
        }

        return ret;
    }
    // found node
    else
    {
        auto running_node = it->second;

        // exact same config
        if(*msg == *running_node.configuration)
        {
            RCLCPP_INFO(this->get_logger(), "Node with this id and configuration is already running, ignoring");
            return 02;
        }
        // change in parameter
        else if(msg->param_json != running_node.configuration->param_json)
        {
            std::vector<std::string> new_params = ParseJson(msg->param_json);

            ParameterChangeOnNode(running_node.configuration->node_name, new_params);

            return 04;
        }
        // restart
        else
        {
            DeleteNode(running_node.configuration);
            return CreateNode(msg);
        }
    }
}

int NodeManager::DeleteNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg) 
{
    auto it = nodes.find(msg->node_id);

    if (it != nodes.end())
    {
        RCLCPP_INFO(this->get_logger(), "Deleting node");

        auto node = it->second;

        node.process->Stop();
        
        nodes.erase(it);

        return 0;
    }
    else 
    {
        std::cerr << "This node does not exist" << std::endl;
        return 1;
    }
}