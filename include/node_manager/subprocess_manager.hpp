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
    std::thread stdout_thread;
};


class SubprocessManager
{
private:
    std::unordered_map<std::string, process> processes;

public:
    SubprocessManager();
    ~SubprocessManager();

    template<typename F, typename... FParams>
    void StartProcess(std::string id, std::string command, std::vector<std::string> command_params, F func, std::tuple<FParams...> func_params)
    {
        if (processes.find(id) != processes.end())
        {
            return;
        }

        process &p = processes[id];

        p.child = boost::process::child(boost::process::search_path(command), boost::process::args(command_params),
            (boost::process::std_out & boost::process::std_err) > p.stdout_stream
        );

        p.stdout_thread = std::thread(func, std::ref(p.stdout_stream), func_params);
    }

    void StopProcess(std::string id);

    template<typename F, typename... FParams>
    void RestartProcess(std::string id, std::string command, std::vector<std::string> command_params, F func, std::tuple<FParams...> func_params);

    // void StartNode(std::string id, std::string package_name, std::string node_name, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);
    // void StopNode(std::string node_name);
};