#include "node_manager/node_manager.hpp"

NodeManager::NodeManager() : Node("node_manager")
{
    this->declare_parameter("manager_id", "");
    manager_id = this->get_parameter("manager_id").as_string();

    auto qos = rclcpp::SystemDefaultsQoS()
    .keep_last(10) // history
    .reliable() // reliability
    .durability_volatile(); // durability

    command_subscriber = this->create_subscription<node_manager::msg::DictionarySerialized>("/management/commands", 10, std::bind(&NodeManager::command_callback, this, std::placeholders::_1));
    command_publisher = this->create_publisher<node_manager::msg::DictionarySerialized>("/management/commands", qos);

    RCLCPP_INFO(get_logger(), "NodeManager initialization done.");
}

NodeManager::~NodeManager() 
{
    RCLCPP_INFO(get_logger(), "NodeManager shutting down and stopping all currently running nodes");

    for (const auto& [id, node]: nodes)
    {
        delete_node(node.config);
    }
}

void NodeManager::command_callback(node_manager::msg::DictionarySerialized::ConstSharedPtr ros_msg) 
{
    std::unordered_map<std::string, std::string> msg = deserialize_command_message(ros_msg);

    if (msg["manager_id"] != this->manager_id)
    {
        RCLCPP_INFO(get_logger(), "Recieved command for someone else, ignoring");
        return;
    }

    // create node
    if (msg["message_type"] == "10" || msg["message_type"] == "20")
    {
        int res = create_node(msg);

        std::unordered_map<std::string, std::string> reply;
        reply["manager_id"] = this->manager_id;
        reply["message_type"] = "90";
        reply["node_id"] = msg["node_id"];
        reply["return_value"] = res;

        node_manager::msg::DictionarySerialized ros_reply = serialize_command_message(reply);
        this->command_publisher->publish(ros_reply);
    }
    // call script
    else if (msg["message_type"] == "30")
    {
        std::string json_output = call_python_script(msg);

        std::unordered_map<std::string, std::string> reply;
        reply["manager_id"] = this->manager_id;
        reply["message_type"] = "90";
        reply["node_id"] = msg["node_id"];
        json_string_to_dictionary(json_output, reply);

        node_manager::msg::DictionarySerialized ros_reply = serialize_command_message(reply);
        this->command_publisher->publish(ros_reply);
    }
    // delete node
    else if (msg["message_type"] == "50")
    {
        int res = delete_node(msg);

        std::unordered_map<std::string, std::string> reply;
        reply["manager_id"] = this->manager_id;
        reply["message_type"] = "90";
        reply["node_id"] = msg["node_id"];
        reply["return_value"] = res;

        node_manager::msg::DictionarySerialized ros_reply = serialize_command_message(reply);
        this->command_publisher->publish(ros_reply);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "I heard unknown message type\n");
    }
}

int NodeManager::create_node(std::unordered_map<std::string, std::string> msg)
{
    auto it = nodes.find(msg["node_id"]);

    // node does not exist yet
    if (it == nodes.end())
    {
        RCLCPP_INFO(get_logger(), "Creating a new node");

        NodeStruct node;
        node.config = msg;

        std::vector<std::string> command_params;

        if (node.config["message_type"] == "10") 
        {
            command_params.push_back("run");
            command_params.push_back(node.config["package_name"]);
            command_params.push_back(node.config["node_name"]);
            command_params.push_back("--ros-args");

            for (const auto &[key, value] : node.config)
            {
                if (key.substr(0,5) == "param")
                {
                    command_params.push_back("-p");
                    command_params.push_back(key.substr(6) + ":=" + value);
                }
            }

            for (const auto &[key, value] : node.config)
            {
                if (key.substr(0,5) == "remap")
                {
                    command_params.push_back("-r");
                    command_params.push_back(key.substr(6) + ":=" + value);
                }
            }
        }
        else if (node.config["message_type"] == "20") 
        {
            command_params.push_back("launch");
            command_params.push_back(node.config["package_name"]);
            command_params.push_back(node.config["node_name"]);

            for (const auto &[key, value] : node.config)
            {
                if (key.substr(0,3) == "arg")
                {
                    command_params.push_back(key.substr(4) + ":=" + value);
                }
            }
        }

        for (auto i : command_params)
        {
            RCLCPP_INFO(get_logger(), "%s", i.c_str());
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::String>("/management/stdout/" + node.config["node_id"], 5);

        node.process = std::make_shared<SubprocessWithRosPublisher>("ros2", command_params, publisher);

        int ret = node.process->Start();
        if (ret == 01) {
            nodes[node.config["node_id"]] = node;
        }

        return ret;
    }
    // found node
    else
    {
        auto running_node = it->second;

        // exact same config
        if(msg == running_node.config)
        {
            RCLCPP_INFO(get_logger(), "Node with this id and configuration is already running, ignoring");
            return 11;
        }
        // change in parameter
        // else if(msg->param_json != running_node.configuration->param_json)
        // {
        //     // std::vector<std::string> new_params = ParseJson(msg->param_json);

        //     // ParameterChangeOnNode(running_node.configuration->node_name, new_params);

        //     // todo return values here
        //     return 04;
        // }
        // restart
        else
        {
            delete_node(running_node.config);
            return create_node(msg);
        }
    }
}

int NodeManager::delete_node(std::unordered_map<std::string, std::string> msg) 
{
    auto it = nodes.find(msg["node_id"]);

    if (it != nodes.end())
    {
        RCLCPP_INFO(get_logger(), "Deleting node %s with id %d", msg["node_name"], msg["node_id"]);

        auto node = it->second;

        int res = node.process->Stop();
        
        nodes.erase(it);

        return res;
    }
    else 
    {
        RCLCPP_WARN(get_logger(), "Node with name %s and id %d does not exist and therefore cannot be deleted", msg["node_name"], msg["node_id"]);
        return 15;
    }
}

std::string NodeManager::call_python_script(std::unordered_map<std::string, std::string> msg)
{
    RCLCPP_INFO(get_logger(), "Calling Python script");

    std::vector<std::string> command_params;
    std::string path = ament_index_cpp::get_package_share_directory("node_manager") + "/scripts/" + msg["script_name"] + ".py";
    command_params.push_back(path);

    // params
    if (msg["script_name"] == "list_nodes" || msg["script_name"] == "list_launch")
    {
        command_params.push_back(msg["package_name"]);   
    }
    if (msg["script_name"] == "get_launch_arg")
    {
        command_params.push_back(msg["launch_file_path"]);   
    }
    if (msg["script_name"] == "get_interface_path")
    {
        command_params.push_back(msg["interface_name"]);   
    }
    if (msg["script_name"] == "get_full_launch_path")
    {
        command_params.push_back(msg["package_name"]);   
        command_params.push_back(msg["launch_file_name"]);   
    }
    if (msg["script_name"] == "get_file_content")
    {
        command_params.push_back(msg["launch_file_path"]);   
    }

    for (const auto s : command_params)
    {
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
    }

    std::shared_ptr<SubprocessWithGetterOutput> process = std::make_shared<SubprocessWithGetterOutput>("python3", command_params);
    int ret = process->Start();
    std::string result = process->get_result();

    return result;
}

std::unordered_map<std::string, std::string> NodeManager::deserialize_command_message(node_manager::msg::DictionarySerialized::ConstSharedPtr msg)
{
    std::unordered_map<std::string, std::string> dict;

    for (const auto &entry: msg->data) {
        dict[entry.key] = entry.value;
    }

    return dict;
}

node_manager::msg::DictionarySerialized NodeManager::serialize_command_message(std::unordered_map<std::string, std::string> dict)
{
    node_manager::msg::DictionarySerialized msg;

    for (const auto &[key, value] : dict)
    {
        msg.data.emplace_back();
        msg.data.back().key = key;
        msg.data.back().value = value;
    }

    return msg;
}

void NodeManager::json_string_to_dictionary(std::string json_string, std::unordered_map<std::string, std::string> &dictionary)
{
    const auto jv = boost::json::parse(json_string);

    if (jv.is_array())
    {
        const auto arr = jv.as_array();

        for (int i = 0; i < arr.size(); i++)
        {
            dictionary[std::to_string(i)] = arr[i].as_string().c_str();
        }
    }

    if (jv.is_object())
    {
        const auto obj = jv.as_object();

        for (const auto [key, value] : obj)
        {
            dictionary[key] = value.as_string().c_str();
        }
    }
}

// int NodeManager::ParameterChangeOnNode(std::string node_name, std::vector<std::string> params)
// {
//     auto client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);

//     if (!client->wait_for_service(std::chrono::seconds(1))) {
//         RCLCPP_ERROR(get_logger(), "Parameter service not available on node %s", node_name);
//         return 1;
//     }

//     for (auto param : params)
//     {
//         auto pos = param.find(":=");
//         if (pos != std::string::npos) 
//         {
//             std::string name  = param.substr(0, pos);
//             std::string value = param.substr(pos + 2);

//             auto results = client->set_parameters({rclcpp::Parameter(name, value)});

//             for (auto & result : results)
//             {
//                 if (!result.successful) 
//                 {
//                     RCLCPP_WARN(get_logger(), "Failed to set param %s: %s", name, result.reason.c_str());
//                     return 1;
//                 }   
//             }
//         }
//     }

//     return 0;
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<NodeManager> node_manager = std::make_shared<NodeManager>();
    
    rclcpp::spin(node_manager);
    
    rclcpp::shutdown();
    
    return 0;
}