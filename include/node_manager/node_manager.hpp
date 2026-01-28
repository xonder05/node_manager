#pragma once

// C
#include <sys/select.h>
#include <unistd.h>

// CPP
#include <string>
#include <memory>
#include <thread>
#include <boost/json.hpp>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Package
#include "node_manager/msg/dictionary_serialized.hpp"
#include "node_manager/subprocess_manager.hpp"


class NodeManager : public rclcpp::Node
{
public:

    NodeManager();
    ~NodeManager();

    // todo lifecycle commands?

private:

    struct NodeStruct {
        std::unordered_map<std::string, std::string> config;
        std::shared_ptr<SubprocessWithRosPublisher> process;
    };
    std::unordered_map<std::string, NodeStruct> nodes;

    // ROS2
    std::string manager_id;

    rclcpp::Subscription<node_manager::msg::DictionarySerialized>::SharedPtr command_subscriber;
    rclcpp::Publisher<node_manager::msg::DictionarySerialized>::SharedPtr command_publisher;

    void command_callback(node_manager::msg::DictionarySerialized::ConstSharedPtr msg);

    // commands
    int create_node(std::unordered_map<std::string, std::string> msg);
    int delete_node(std::unordered_map<std::string, std::string> msg);
    std::string call_python_script(std::unordered_map<std::string, std::string> msg);

    // helpers
    std::unordered_map<std::string, std::string> deserialize_command_message(node_manager::msg::DictionarySerialized::ConstSharedPtr msg);
    node_manager::msg::DictionarySerialized serialize_command_message(std::unordered_map<std::string, std::string> msg);
    void json_string_to_dictionary(std::string json_string, std::unordered_map<std::string, std::string>& dictionary);

    // int ParameterChangeOnNode(std::string node_name, std::vector<std::string> params);
};
