#pragma once

#include <boost/process.hpp>
#include <iostream>
#include <thread>
#include <csignal>
#include <unistd.h>
#include <unordered_map>
#include <string>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>

struct process
{
    boost::process::child child;
    boost::process::ipstream stdout_stream;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    std::thread publisher_thread;
};


class SubprocessManager
{
private:
    std::unordered_map<std::string, process> processes;

    static void stream_thread(boost::process::ipstream& stream, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher) 
    {
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

public:
    SubprocessManager(/* args */);
    ~SubprocessManager();

    void StartNode(std::string package_name, std::string node_name, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);
    void StopNode(std::string node_name);
};