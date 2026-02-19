#include "node_manager/node_manager.hpp"

NodeManager::NodeManager() : Node("node_manager")
{
    this->declare_parameter("manager_id", "");
    manager_id = this->get_parameter("manager_id").as_string();

    auto qos = rclcpp::SystemDefaultsQoS()
    .keep_last(10) // history
    .reliable() // reliability
    .durability_volatile(); // durability

    commands = this->create_service<node_manager::srv::DictionarySerialized>("commands", std::bind(&NodeManager::command_callback, this, std::placeholders::_1, std::placeholders::_2));

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

void NodeManager::command_callback(node_manager::srv::DictionarySerialized::Request::ConstSharedPtr req, node_manager::srv::DictionarySerialized::Response::SharedPtr res) 
{
    std::unordered_map<std::string, std::string> msg;
    deserialize_command_message(req, msg);

    if (msg["manager_id"] != this->manager_id)
    {
        RCLCPP_INFO(get_logger(), "Recieved command for someone else, ignoring");
        return;
    }

    // create node
    if (msg["message_type"] == "10" || msg["message_type"] == "20")
    {
        int ret = create_node(msg);

        std::unordered_map<std::string, std::string> reply = create_reply_message(msg);;
        reply["return_value"] = std::to_string(ret);

        serialize_command_message(reply, res);
    }
    
    // call script
    else if (msg["message_type"] == "30")
    {
        auto [ret, json_output] = call_python_script(msg);

        std::unordered_map<std::string, std::string> reply = create_reply_message(msg);;
        json_string_to_dictionary(json_output, reply);
        reply["return_value"] = std::to_string(ret);

        serialize_command_message(reply, res);
    }

    // read file
    else if (msg["message_type"] == "40")
    {
        std::unordered_map<std::string, std::string> reply = create_reply_message(msg);;

        auto [ret, json_output] = call_python_script(msg);

        if (ret == 03)
        {
            std::unordered_map<std::string, std::string> script_output;
            json_string_to_dictionary(json_output, script_output);

            try
            {
                std::string file_content = read_file(script_output["file_path"]);
                reply["file_content"] = file_content;
                reply["return_value"] = std::to_string(04);
            }
            catch(const std::runtime_error& e) 
            {
                reply["return_value"] = std::to_string(24);
            }
        }
        else
        {
            reply["return_value"] = std::to_string(ret);
        }

        serialize_command_message(reply, res);
    }

    // write file
    else if (msg["message_type"] == "41")
    {
        std::unordered_map<std::string, std::string> reply = create_reply_message(msg);;

        auto [ret, json_output] = call_python_script(msg);

        if (ret == 03)
        {
            std::unordered_map<std::string, std::string> script_output;
            json_string_to_dictionary(json_output, script_output);

            try 
            {
                write_file(script_output["file_path"], msg["file_content"]);
                reply["return_value"] = std::to_string(04);
            }
            catch(const std::runtime_error& e) 
            {
                reply["return_value"] = std::to_string(24);
            }
        }
        else
        {
            reply["return_value"] = std::to_string(ret);
        }

        serialize_command_message(reply, res);
    }

    // delete node
    else if (msg["message_type"] == "50")
    {
        int ret = delete_node(msg);

        std::unordered_map<std::string, std::string> reply;
        reply["manager_id"] = this->manager_id;
        reply["message_type"] = "90";
        reply["node_id"] = msg["node_id"];
        reply["return_value"] = std::to_string(ret);

        serialize_command_message(reply, res);
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

            if (node.config.find("runtime_node_name") != node.config.end())
            {
                command_params.push_back("-r");
                command_params.push_back("__node:=" + node.config["runtime_node_name"]);
            }

            if (node.config.find("namespace") != node.config.end())
            {
                command_params.push_back("-r");
                command_params.push_back("__ns:=" + node.config["namespace"]);
            }

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

        // same config
        if(msg == running_node.config)
        {
            RCLCPP_INFO(get_logger(), "Node with this id and configuration is already running, ignoring");
            return 11;
        }
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

        auto [res, status] = node.process->Stop();
        
        nodes.erase(it);

        return res;
    }
    else 
    {
        RCLCPP_WARN(get_logger(), "Node with name %s and id %d does not exist and therefore cannot be deleted", msg["node_name"], msg["node_id"]);
        return 15;
    }
}

std::tuple<int, std::string> NodeManager::call_python_script(std::unordered_map<std::string, std::string> msg)
{
    RCLCPP_INFO(get_logger(), "Calling Python script");

    std::vector<std::string> command_params;
    std::string path = ament_index_cpp::get_package_share_directory("node_manager") + "/scripts/" + msg["script_name"] + ".py";
    command_params.push_back(path);

    // params
    if (msg["script_name"] == "list_nodes" || msg["script_name"] == "list_launch_files" || msg["script_name"] == "list_config_files")
    {
        command_params.push_back(msg["package_name"]);
    }
    if (msg["script_name"] == "resolve_file_path" || msg["script_name"] == "list_launch_arguments")
    {
        command_params.push_back(msg["package_name"]);   
        command_params.push_back(msg["file_name"]);   
    }

    for (const auto s : command_params)
    {
        RCLCPP_INFO(get_logger(), "%s", s.c_str());
    }

    std::shared_ptr<SubprocessWithGetterOutput> process = std::make_shared<SubprocessWithGetterOutput>("python3", command_params);
    process->Start();
    std::string result = process->get_result();
    auto [ret, status] = process->Stop();

    if (ret != 05)
    {
        return std::make_tuple(ret, "{}");
    }
    else if (WEXITSTATUS(status) != 0) 
    {
        return std::make_tuple(23, "{}");
    }
    else
    {
        return std::make_tuple(03, result);
    }
}

std::unordered_map<std::string, std::string> NodeManager::create_reply_message(std::unordered_map<std::string, std::string> msg)
{
    std::unordered_map<std::string, std::string> reply;
    reply["message_type"] = "90";
    reply["manager_id"] = this->manager_id;
    reply["node_id"] = msg["node_id"];

    return reply;
}

void NodeManager::deserialize_command_message(node_manager::srv::DictionarySerialized::Request::ConstSharedPtr msg, std::unordered_map<std::string, std::string>& dict)
{
    for (const auto &entry: msg->data) 
    {
        dict[entry.key] = entry.value;
    }
}

void NodeManager::serialize_command_message(std::unordered_map<std::string, std::string> dict, node_manager::srv::DictionarySerialized::Response::SharedPtr msg)
{
    for (const auto &[key, value] : dict)
    {
        msg->data.emplace_back();
        msg->data.back().key = key;
        msg->data.back().value = value;
    }
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

std::string NodeManager::read_file(std::string file_path)
{
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file (ifstream)");
    }

    return std::string(
        (std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>()
    );
}

void NodeManager::write_file(std::string file_path, std::string file_content)
{
    std::ofstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Cannot open file (ofstream)");
    }

    file.write(file_content.data(), file_content.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<NodeManager> node_manager = std::make_shared<NodeManager>();
    
    rclcpp::spin(node_manager);
    
    rclcpp::shutdown();
    
    return 0;
}