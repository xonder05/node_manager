#pragma once

// C
#include <sys/select.h>
#include <unistd.h>

// CPP
#include <string>
#include <memory>
#include <thread>
#include <boost/json.hpp>
#include <fstream>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Package
#include "node_manager/srv/dictionary_serialized.hpp"
#include "node_manager/subprocess_manager.hpp"


class NodeManager : public rclcpp::Node
{
public:

    NodeManager();
    ~NodeManager();

private:

    struct NodeStruct {
        std::unordered_map<std::string, std::string> config;
        std::shared_ptr<SubprocessWithRosPublisher> process;
    };
    std::unordered_map<std::string, NodeStruct> nodes;

    // ROS2
    std::string manager_id;

    rclcpp::Service<node_manager::srv::DictionarySerialized>::SharedPtr commands;

    void command_callback(node_manager::srv::DictionarySerialized::Request::ConstSharedPtr req, node_manager::srv::DictionarySerialized::Response::SharedPtr res);

    // commands
    int create_node(std::unordered_map<std::string, std::string> msg);
    int delete_node(std::unordered_map<std::string, std::string> msg);
    std::tuple<int, std::string> call_python_script(std::unordered_map<std::string, std::string> msg);

    // helpers
    std::unordered_map<std::string, std::string> create_reply_message(std::unordered_map<std::string, std::string> msg);
    void deserialize_command_message(node_manager::srv::DictionarySerialized::Request::ConstSharedPtr msg, std::unordered_map<std::string, std::string>& dict);
    void serialize_command_message(std::unordered_map<std::string, std::string> dict, node_manager::srv::DictionarySerialized::Response::SharedPtr msg);
    void json_string_to_dictionary(std::string json_string, std::unordered_map<std::string, std::string>& dictionary);
    std::string read_file(std::string file_path);
    void write_file(std::string file_path, std::string file_content);
};
