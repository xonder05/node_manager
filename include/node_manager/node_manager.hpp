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

class NodeManager : public rclcpp::Node
{
public:

    NodeManager();
    ~NodeManager();

    void RunNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    void StopNode(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    void RunLaunchFile();

    // todo lifecycle commands?

private:

    std::unordered_map<std::string, node_manager::msg::NodeRedCommand::ConstSharedPtr> nodes;

    SubprocessManager subprocess_manager;

    rclcpp::Subscription<node_manager::msg::NodeRedCommand>::SharedPtr command_subscriber;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void commandCallback(node_manager::msg::NodeRedCommand::ConstSharedPtr msg);
    std::vector<std::string> ParseJson(std::string json);

    static void stream_thread(boost::process::ipstream& stream, std::tuple<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> args_tuple) 
    {
        auto [publisher] = args_tuple;

        try
        {
            std::cout << "starting thread" << std::endl;

            std::string line;
            std_msgs::msg::String msg;
            
            while (std::getline(stream, line)) 
            {
                msg.data = line;
                publisher->publish(msg);
            }
            std::cout << "stopping thread" << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

};
