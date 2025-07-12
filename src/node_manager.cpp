#include "node_manager/node_manager.hpp"

NodeManager::NodeManager() : Node("node_manager")
{
    auto qos = rclcpp::SystemDefaultsQoS()
    .keep_last(10) // history
    .reliable() // reliability
    .transient_local(); // durability

    // publisher_ = this->create_publisher<std_msgs::msg::String>
    //     ("/management/agent_advertisement", qos);

    // std_msgs::msg::String msg;
    // msg.data = "this is my id";
    // this->publisher_->publish(msg);

    // todo get presistent manager id from parameter

    this->declare_parameter("manager_id", "");
    manager_id = this->get_parameter("manager_id").as_string();

    command_subscriber = this->create_subscription<node_manager::msg::NodeRedCommand>("/management/commands", 10, std::bind(&NodeManager::commandCallback, this, std::placeholders::_1));
    command_publisher = this->create_publisher<node_manager::msg::NodeRedCommand>("/management/commands", qos);

    timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
            });

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
            int res = RunNode(msg);
            node_manager::msg::NodeRedCommand reply;
            
            reply.manager_id = this->manager_id;
            reply.message_type = 99;
            reply.node_id = msg->node_id;
            reply.return_value = res;

            this->command_publisher->publish(reply);
            break;
        }
        
        case 1:
        {
            int res = StopNode(msg);

            node_manager::msg::NodeRedCommand reply;
            
            reply.manager_id = this->manager_id;
            reply.message_type = 99;
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

void NodeManager::stream_thread(process& p, std::tuple<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> args_tuple) 
{
    // get unix file descriptor
    int fd = p.stdout_stream.pipe().native_source();

    // prepare for select()
    fd_set set;
    struct timeval timeout;

    // prepare for reading data
    std::string line;
    std_msgs::msg::String msg;
    auto [publisher] = args_tuple;

    // reading loop
    while (p.child.running()) 
    {
        FD_ZERO(&set);
        FD_SET(fd, &set);
        
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(fd + 1, &set, NULL, NULL, &timeout);

        // no data received
        if (ret == 0) 
        {
            continue;
        }
        // error
        else if (ret < 0)
        {
        }
        // got data
        else 
        {
            std::getline(p.stdout_stream, line);
            msg.data = line;
            publisher->publish(msg);
        }
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

int NodeManager::RunNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg)
{
    // start new node
    if (nodes.find(msg->node_id) == nodes.end())
    {
        RCLCPP_INFO(this->get_logger(), "Starting node");

        std::vector<std::string> command_params;
        command_params.push_back("run");
        command_params.push_back(msg->package_name);
        command_params.push_back(msg->node_name);
        
        auto node_params = ParseJson(msg->param_json);
        command_params.insert(command_params.end(), node_params.begin(), node_params.end());

        auto node_remaps = ParseJson(msg->remap_json);
        command_params.insert(command_params.end(), node_remaps.begin(), node_remaps.end());

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::String>("/management/stdout/" + msg->node_id, 5);

        int ret = subprocess_manager.StartProcess(msg->node_id, "ros2", command_params, stream_thread, std::make_tuple(publisher));

        // process started succesfully
        if (ret < 10) 
        {
            node n;
            n.configuration = msg;
            n.stdout_publisher = publisher;

            nodes[msg->node_id] = n;
        }

        return ret;
    }
    // node already running
    else
    {
        auto running_node = nodes.at(msg->node_id);

        // exact same config
        if(*msg == *running_node.configuration)
        {
            RCLCPP_INFO(this->get_logger(), "Node with this id and configuration is already running, ignoring");
            return 02;
        }
        // change in parameter
        else if(msg->param_json != running_node.configuration->param_json)
        {
            // boost::json::value val = boost::json::parse(msg->param_json);

            // if(!val.is_array())
            // {
            //     RCLCPP_WARN(this->get_logger(), "Params json is not array, ignoring");
            //     return 1;
            // }

            // boost::json::array params = val.as_array();
            // std::string params_string;
            // for (const auto &param : params)
            // {
            //     if(param.is_string()) {
            //         params_string.append(param.as_string());
            //     }
            // }

            // StopNode(running_node.configuration);
            return 04;
        }
        // everything else requires restart
        else
        {
            StopNode(running_node.configuration);
            return RunNode(msg);
        }
    }
}

int NodeManager::StopNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg) 
{
    RCLCPP_INFO(this->get_logger(), "Stopping node");

    int ret = subprocess_manager.StopProcess(msg->node_id);

    RCLCPP_INFO(this->get_logger(), "ret: %d", ret);

    if (ret < 10) {
        nodes.erase(msg->node_id);
    }   

    return ret;
}

int NodeManager::RunLaunchFile()
{
    return 3;
}
