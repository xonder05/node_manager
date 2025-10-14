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

class NodeManager : public rclcpp::Node
{
public:

    NodeManager();
    ~NodeManager();

    int CreateNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    int DeleteNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);

    // todo lifecycle commands?

private:

    std::string manager_id;

    struct node
    {
        node_manager::msg::NodeRedCommand::ConstSharedPtr configuration;
        std::shared_ptr<SubprocessWithRosPublisher> process;
    };

    std::unordered_map<std::string, node> nodes;

    rclcpp::Subscription<node_manager::msg::NodeRedCommand>::SharedPtr command_subscriber;
    rclcpp::Publisher<node_manager::msg::NodeRedCommand>::SharedPtr command_publisher;

    void commandCallback(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    std::vector<std::string> ParseJson(std::string json);
    int ParameterChangeOnNode(std::string node_name, std::vector<std::string> params);
};
