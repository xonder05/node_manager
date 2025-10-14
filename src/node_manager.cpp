#include "node_manager/node_manager.hpp"

NodeManager::NodeManager() : Node("node_manager")
{
    auto qos = rclcpp::SystemDefaultsQoS()
    .keep_last(10) // history
    .reliable() // reliability
    .durability_volatile(); // durability

    this->declare_parameter("manager_id", "");
    manager_id = this->get_parameter("manager_id").as_string();

    command_subscriber = this->create_subscription<node_manager::msg::NodeRedCommand>("/management/commands", 10, std::bind(&NodeManager::commandCallback, this, std::placeholders::_1));
    command_publisher = this->create_publisher<node_manager::msg::NodeRedCommand>("/management/commands", qos);

    RCLCPP_INFO(get_logger(), "NodeManager initialization done.");
}

NodeManager::~NodeManager() 
{
    RCLCPP_INFO(get_logger(), "NodeManager shutting down and stopping all currently running nodes");

    for (const auto& [id, node]: nodes)
    {
        DeleteNode(node.configuration);
    }
}

void NodeManager::commandCallback(node_manager::msg::NodeRedCommand::ConstSharedPtr msg) 
{
    if (msg->manager_id != this->manager_id)
    {
        RCLCPP_INFO(get_logger(), "Recieved command for someone else, ignoring");
        return;
    }

    switch (msg->message_type)
    {
        case 10: case 20:
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
        
        case 50:
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

        default:
            RCLCPP_INFO(get_logger(), "I heard unknown message type\n");
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
        RCLCPP_WARN(get_logger(), "Parameter / Remap is not in json format, ignoring");
        return std::vector<std::string>();
    }

    if(!json_value.is_object())
    {
        RCLCPP_WARN(get_logger(), "Parameter / Remap json is not object, ignoring");
        return std::vector<std::string>();
    }

    boost::json::object json_obj = json_value.as_object();

    std::vector<std::string> command_parameters;

    for (auto& [key, value] : json_obj)
    {
        command_parameters.push_back(std::string(key) + ":=" + std::string(value.as_string()));
    }

    return command_parameters;
}

int NodeManager::ParameterChangeOnNode(std::string node_name, std::vector<std::string> params)
{
    auto client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);

    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "Parameter service not available on node %s", node_name);
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
                    RCLCPP_WARN(get_logger(), "Failed to set param %s: %s", name, result.reason.c_str());
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
        RCLCPP_INFO(get_logger(), "Creating a new node");

        node n;
        n.configuration = msg;

        std::vector<std::string> command_params;

        if (n.configuration->message_type == 10) 
        {
            command_params.push_back("run");
            command_params.push_back(n.configuration->package_name);
            command_params.push_back(n.configuration->node_name);
            command_params.push_back("--ros-args");

            auto node_params = ParseJson(n.configuration->param_json);
            if (!node_params.empty())
            {
                for(auto param : node_params)
                {
                    command_params.push_back("-p");
                    command_params.push_back(param);
                }
            }

            auto node_remaps = ParseJson(n.configuration->remap_json);
            if (!node_remaps.empty())
            {
                for(auto remap : node_remaps)
                {
                    command_params.push_back("-r");
                    command_params.push_back(remap);
                }
            }
        }
        else if (n.configuration->message_type == 20) 
        {
            command_params.push_back("launch");
            command_params.push_back(n.configuration->package_name);
            command_params.push_back(n.configuration->node_name);

            auto launch_args = ParseJson(n.configuration->param_json);
            if (!launch_args.empty())
            {
                for(auto arg : launch_args)
                {
                    command_params.push_back(arg);
                }
            }
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::String>("/management/stdout/" + n.configuration->node_id, 5);

        n.process = std::make_shared<SubprocessWithRosPublisher>("ros2", command_params, publisher);

        int ret = n.process->Start();
        if (ret == 01) {
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
            RCLCPP_INFO(get_logger(), "Node with this id and configuration is already running, ignoring");
            return 11;
        }
        // change in parameter
        else if(msg->param_json != running_node.configuration->param_json)
        {
            // std::vector<std::string> new_params = ParseJson(msg->param_json);

            // ParameterChangeOnNode(running_node.configuration->node_name, new_params);

            // todo return values here
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
        RCLCPP_INFO(get_logger(), "Deleting node %s with id %d", msg->node_name, msg->node_id);

        auto node = it->second;

        int res = node.process->Stop();
        
        nodes.erase(it);

        return res;
    }
    else 
    {
        RCLCPP_WARN(get_logger(), "Node with name %s and id %d does not exist and therefore cannot be deleted", msg->node_name, msg->node_id);
        return 15;
    }
}
