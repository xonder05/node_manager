#include "node_manager/subprocess_manager.hpp"

SubprocessManager::SubprocessManager() { }
SubprocessManager::~SubprocessManager() 
{
    for (auto& pair : processes)
    {
        std::cout << "stopping " << pair.first << std::endl;
        StopNode(pair.first);
    }
}

void SubprocessManager::StartNode(std::string id, std::string package_name, std::string node_name, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
    if (processes.find(id) != processes.end())
    {
        return;
    }

    process &p = processes[id];

    p.child = boost::process::child(
        boost::process::search_path("ros2"), "run", package_name, node_name,
        (boost::process::std_out & boost::process::std_err) > p.stdout_stream
    );

    p.publisher = publisher;
    p.publisher_thread = std::thread(stream_thread, std::ref(p.stdout_stream), p.publisher);
}

void SubprocessManager::StopNode(std::string node_name)
{
    std::string identifier = node_name + "1";
    if (processes.find(identifier) != processes.end())
    {
        process &p = processes[identifier];

        // guard, you have no idea how many times i killed everyhting with pid -1
        pid_t pid = p.child.id();
        if (pid <= 1) {
            return;
        }

        kill(pid, SIGTERM);
        p.child.wait();
        std::cout << "Process succesfully stopped with return code " << p.child.exit_code() << std::endl;

        auto thread = std::move(p.publisher_thread);
        processes.erase(identifier);

        if (!thread.joinable())
        {
            std::cout << "The thread is already done, which it should not be." << std::endl;
            return;
        }
        thread.join();

        std::cout << "done" << std::endl;
    }
}