#pragma once

#include <string>
#include <memory>
#include <thread>
#include <boost/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"

#include "node_manager/msg/node_red_command.hpp"
#include "node_manager/subprocess_manager.hpp"

#include <sys/select.h>
#include <unistd.h>

struct node
{
    node_manager::msg::NodeRedCommand::ConstSharedPtr configuration;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stdout_publisher;
};


class NodeManager : public rclcpp::Node
{
public:

    NodeManager();
    ~NodeManager();

    int RunNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    int StopNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    int RunLaunchFile();

    // todo lifecycle commands?

private:

    std::string manager_id;

    std::unordered_map<std::string, node> nodes;

    SubprocessManager subprocess_manager;

    rclcpp::Subscription<node_manager::msg::NodeRedCommand>::SharedPtr command_subscriber;
    rclcpp::Publisher<node_manager::msg::NodeRedCommand>::SharedPtr command_publisher;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void commandCallback(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    std::vector<std::string> ParseJson(std::string json);

    static void stream_thread(process& p, std::tuple<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> args_tuple);

};
