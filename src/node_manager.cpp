#include "node_manager/node_manager.hpp"

NodeManager::NodeManager() : Node("node_manager")
{
    // auto qos = rclcpp::SystemDefaultsQoS()
    // .keep_last(1) // history
    // .reliable() // reliability
    // .transient_local(); // durability

    // publisher_ = this->create_publisher<std_msgs::msg::String>
    //     ("/management/agent_advertisement", qos);

    // std_msgs::msg::String msg;
    // msg.data = "this is my id";
    // this->publisher_->publish(msg);

    // todo get presistent manager id from parameter

    command_subscriber = this->create_subscription<node_manager::msg::NodeRedCommand>("/management/commands", 10, std::bind(&NodeManager::commandCallback, this, std::placeholders::_1));

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
    if (msg->manager_id != 1) 
    {
        RCLCPP_INFO(this->get_logger(), "Recieved command for someone else, ignoring");
        return;
    }

    switch (msg->message_type)
    {
        case 0:
        {
            RunNode(msg);
            break;
        }
        
        case 1:
        {
            StopNode(msg);
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

void NodeManager::RunNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg)
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

        subprocess_manager.StartProcess(msg->node_id, "ros2", command_params, stream_thread, std::make_tuple(publisher));
    }
    // node already running
    else
    {
        auto running_node = nodes.at(msg->node_id);

        if(msg == running_node)
        {
            RCLCPP_INFO(this->get_logger(), "Node with this id and configuration is already running, ignoring");
            return;
        }

        // todo add check so it does not lead to infinte recursion in case of error
        if(msg->package_name != running_node->package_name || msg->node_name != running_node->node_name)
        {
            StopNode(running_node);
            RunNode(msg);
        }

        if(msg->param_json != running_node->param_json)
        {
            boost::json::value val = boost::json::parse(msg->param_json);

            if(!val.is_array())
            {
                RCLCPP_WARN(this->get_logger(), "Params json is not array, ignoring");
                return;
            }

            boost::json::array params = val.as_array();
            std::string params_string;
            for (const auto &param : params)
            {
                if(param.is_string()) {
                    params_string.append(param.as_string());
                }
            }

            StopNode(running_node);

        }

        if(msg->remap_json != running_node->remap_json)
        {

        }


    }



}

void NodeManager::StopNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg) 
{
    subprocess_manager.StopProcess(msg->node_name);
}

void NodeManager::RunLaunchFile()
{
}
