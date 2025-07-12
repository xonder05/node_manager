/**
 * @file subprocess_manager.hpp
 * @brief Class for managing processes using boost::process library
 * @author Onderka Daniel (xonder05)
 * @date 06/2025
 */

#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <thread>
#include <boost/process.hpp>

#include "std_msgs/msg/string.hpp"

struct process
{
    boost::process::child child;
    boost::process::ipstream stdout_stream;
    std::thread stdout_thread;
};

class SubprocessManager
{
private:

    std::unordered_map<std::string, process> processes;

public:

    /**
     * @brief Empty constructor for SubprocessManager
     */
    SubprocessManager() { }

    /**
     * @brief Stops all managed processes and destroys SubprocessManager object
     */
    ~SubprocessManager() 
    {
        for (auto& pair : processes)
        {
            StopProcess(pair.first);
        }
    }

    /**
     * @brief Starts a new child process and adds a record to processes map
     *
     * @tparam F any function for processing stdout
     * @tparam FParams list of parameters that will be passed to the function of type F
     * 
     * @param id identification of the process, matches node id in manager and node-red
     * @param command process to be run, eg: "ros2, ls, pwd"
     * @param command_params parameters of the command
     * @param func function for processing stdout, will be run in separate thread 
     * @param func_params parameters for the function
     * 
     * @return meaning of return values can be found in /msg/README.md
     */
    template<typename F, typename... FParams>
    int StartProcess(std::string id, std::string command, std::vector<std::string> command_params, F func, std::tuple<FParams...> func_params)
    {
        // process already exists
        if (processes.find(id) != processes.end()) {
            return 21;
        }

        process &p = processes[id];

        p.child = boost::process::child(boost::process::search_path(command), boost::process::args(command_params),
            (boost::process::std_out & boost::process::std_err) > p.stdout_stream
        );

        std::cout << "starting thread" << std::endl;
        p.stdout_thread = std::thread(func, std::ref(p), func_params);

        std::cout << "success in starting a new process" << std::endl;
        // process succesfully started
        return 01;
    }

    /**
     * @brief Kills child process, helper threads and deletes record from processes map 
     *
     * @param id identification of the process, matches node id in manager and node-red

     * @return meaning of return values can be found in /msg/README.md
     */
    int StopProcess(std::string id)
    {
        // process does not exist
        if (processes.find(id) == processes.end()) {
            return 25;
        }
        process& p = processes[id];

        p.child.terminate();
        p.child.wait();
        std::cout << "Subprocess succesfully stopped with return code " << p.child.exit_code() << std::endl;

        if (!p.stdout_thread.joinable())
        {
            std::cout << "Thread is not joinable" << std::endl;
            return 29;
        }
        p.stdout_thread.join();

        processes.erase(id);

        return 05;
    }

    /**
     * @brief Restarts running process
     *
     * @see StopProcess, StartProcess
     */
    template <typename F, typename... FParams>
    int RestartProcess(std::string id, std::string command, std::vector<std::string> command_params, F func, std::tuple<FParams...> func_params)
    {
        int ret = StopProcess(id);
        
        if (ret != 05) {
            return ret;
        }

        return StartProcess(id, command, command_params, func, func_params);
    }

};