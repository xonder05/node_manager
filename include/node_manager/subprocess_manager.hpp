/**
 * @file subprocess_manager.hpp
 * @brief Class representing a single subprocess, handles starting, reading stdout/stderr and gracefull stoping.
 * @author Onderka Daniel (xonder05)
 * @date 09/2025
 */

#pragma once

#include <unistd.h> // fork() execv() pipe()
#include <sys/types.h> // pid_t
#include <sys/wait.h> // waitpid()
#include <sys/stat.h> // stat()
#include <signal.h> // kill()

#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include <chrono>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Subprocess
{
protected:

    pid_t pid;
    int pipes[2];
    std::thread reader_thread;

    std::string executable_name;
    std::string executable_path;
    std::vector<std::string> arguments_string;
    std::vector<char*> arguments_char;

    bool init_success = false;
    bool process_started = false;

    /**
     * @brief Either validates or finds the absolute path to the file (finding using PATH), and checks that it can be executed.
     * 
     * @param executable file name or full path
     * @return full path to valid executable (or empty string in case of error)
     */
    std::string GetFullExecutablePath(const std::string executable)
    {
        // executable is entire path
        if (executable.find('/') != std::string::npos) 
        {
            struct stat st;

            if (stat(executable.c_str(), &st) == -1) {
                perror("stat");
                return "";
            }

            if (st.st_mode & S_IXUSR) {
                return executable;
            } 
            else {
                std::cerr << "Executable file exists but i don't have permission to execute it" << std::endl;
                return "";
            }
        }
        else // executable is just name
        {
            const char *PATH = std::getenv("PATH");
            if (!PATH) {
                return "";  
            } 

            std::stringstream ss(PATH);
            std::string path;

            while (std::getline(ss, path, ':')) 
            {
                std::string executable_path = path + "/" + executable;

                struct stat st;
                if (stat(executable_path.c_str(), &st) == 0 && (st.st_mode & S_IXUSR)) 
                {
                    return executable_path;
                }
            }

            std::cerr << "Executable is either not in PATH or i don't have execute permission" << std::endl;
            return "";
        }
    }

    /**
     * @brief Child's part after calling fork()
     * 
     * creates new group for potential child processes,
     * handles redirection of stdout and stderr into pipe,
     * lastly gives control to child program
     */
    void Child()
    {
        // assign group id to this process, (0, 0) means use pid of active process
        setpgid(0, 0);

        // close read end of pipe, child does not read
        close(pipes[0]);

        // redirect both stdout and stderr of this process into pipe
        dup2(pipes[1], STDOUT_FILENO);
        dup2(pipes[1], STDERR_FILENO);

        // this end has been copied to the stdout and stderr, so it can be closed here
        close(pipes[1]);

        // hand over control to the child binary
        execv(executable_path.c_str(), arguments_char.data());
        perror("execv");
    }

    /**
     * @brief Parent's part after calling fork() 
     * 
     * derived classes should override this and use it to setup a reader thread for child's stdout / stderr,
     * this way derived classes don't have to override the entire Start() method
     * 
     * @return All return values are described in this_package_root/msg/README.md
     */
    virtual int Parent()
    {
        // close write end of the pipe, parent will not write
        close(pipes[1]);


        // todo: figure out if there is a way to check if execv() succeeded
        // one idea would be to create thread that will periodically check that the child is still alive
        return 01;
    }

public:

    /**
     * @brief Constructor
     * 
     * prepares input arguments and evarything else for Start()
     * 
     * @param exec name or path to the executable
     * @param args vector of string arguments
     */
    Subprocess(std::string exec, std::vector<std::string> args) 
    {
        // persistent storage (otherwise stuff breaks when this goes out of scope)
        executable_name = exec;
        arguments_string = args;

        // get full executable path from command name
        executable_path = GetFullExecutablePath(executable_name);
        if (executable_path.empty()) {
            std::cerr << "This: " << executable_name << " is not a valid executable " << std::endl;
            return;
        }
        
        // convert strings to char pointers because of exec()
        // argv[0] == executable name
        arguments_char.push_back(const_cast<char*>(executable_path.c_str()));

        // normal args
        for (auto &a : arguments_string) {
            arguments_char.push_back(const_cast<char*>(a.c_str()));
        }

        // null terminated
        arguments_char.push_back(nullptr);

        // generate pipe, [0] == read end, [1] == write end
        if (pipe(pipes) == -1) { 
            perror("Pipe could not be generated"); 
            return; 
        }
        
        init_success = true;
    }

    /**
     * @brief Destructor
     */
    ~Subprocess() 
    {
        if (process_started)
        {
            Stop();
        }
    }

    /**
     * @brief If constructor succeeded then everything is ready and this just calls fork
     * 
     * @return All return values are described in this_package_root/msg/README.md
     */
    int Start()
    {
        if (!init_success && process_started) {
            return 21;
        }

        pid = fork();
        
        if (pid < 0) {
            perror("fork");
            return 1;
        }

        if (pid == 0) {
            Child();
            exit(1); //child failed in execv()
        }
        else {
            return Parent();
        }
    }

    /**
     * @brief Tries to stop the child process (uses increasing force SIGINT, SIGTERM, SIGKILL)
     * 
     * @param wait [ms], wait is how often will be checked if child exited, after 10 checks signal with increased force will be sent and this will repeat untill sigkill, so max wait time is 30 * wait
     * @return Tuple of ints, first value is return value as described in this_package_root/msg/README.md, second value is status of waitpid call.
     */
    std::tuple<int, int> Stop(int exit_wait_time = 500)
    {
        for (auto level : {SIGINT, SIGTERM, SIGKILL, INT8_MAX})
        {
            bool first = true;

            // pooling for result
            for (int i = 0; i < 10; i++)
            {
                int status;
                pid_t res = waitpid(-pid, &status, WNOHANG);

                if (res == -1) {
                    perror("waitpid");
                    return std::make_tuple(26, -1);
                }
                
                if (res == 0)
                {
                    if (!first) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(exit_wait_time));
                        continue;
                    }

                    if (level == INT8_MAX) {
                        std::cerr << "Child (pid: " << pid << ") is still running even after SIGKILL, ignoring" << std::endl;
                        return std::make_tuple(25, -1);
                    }

                    if (kill(-pid, level) == -1) {
                        perror("kill");
                    }

                    first = false;

                    continue;
                }

                if (WIFEXITED(status)) {
                    std::cout << "Child exited with code " << WEXITSTATUS(status) << std::endl;
                } 
                else if (WIFSIGNALED(status)) {
                    std::cout << "Child killed by signal " << WTERMSIG(status) << std::endl;
                }

                if (reader_thread.joinable()) {
                    reader_thread.join();
                }

                return std::make_tuple(05, status);
            }
        }
    }
};

class SubprocessWithConsoleOutput : public Subprocess
{
protected:

    /**
     * @brief Prints childs output into console
     * 
     * This will be started in separate thread to handle output from the child process
     * 
     * @param fd file descriptor for the pipe where stdout and stderr of child is redirected
     */
    static void ChildOutputHandler(int fd)
    {
        char buffer[256];
        ssize_t n;

        while ((n = read(fd, buffer, sizeof(buffer)-1)) > 0) 
        {
            buffer[n] = '\0';
            std::cout << buffer << std::flush;
        }
    }

    /**
     * @brief Calls base Parent() and starts ChildOutputHandler() in a thread
     * 
     * @return All return values are described in this_package_root/msg/README.md
     */
    int Parent() override
    {
        int ret = Subprocess::Parent();

        reader_thread = std::thread(ChildOutputHandler, pipes[0]);

        return ret;
    }
};

class SubprocessWithRosPublisher : public Subprocess
{
protected:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

    /**
     * @brief Publishes childs output into ros2 topic
     * 
     * This will be started in separate thread to handle output from the child process
     * 
     * @param fd file descriptor for the pipe where stdout and stderr of child is redirected
     * @param publisher pointer to ros2 publisher object
     */
    static void ChildOutputHandler(int fd, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
    {
        std_msgs::msg::String msg;
        char buffer[256];
        ssize_t n;

        while ((n = read(fd, buffer, sizeof(buffer)-1)) > 0) 
        {
            buffer[n] = '\0';
            msg.data = buffer;
            publisher->publish(msg);
        }
    }

    /**
     * @brief Calls base Parent() and starts ChildOutputHandler() in a thread
     * 
     * @return All return values are described in this_package_root/msg/README.md
     */
    int Parent() override
    {
        int ret = Subprocess::Parent();

        reader_thread = std::thread(ChildOutputHandler, pipes[0], publisher);

        return ret;
    }

public:

    /**
     * @brief Same as base constructor, additionally just saves publisher pointer into class atribute
     */
    SubprocessWithRosPublisher(std::string exec, std::vector<std::string> args, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros2_publisher)
    : Subprocess(exec, args)
    {
        publisher = ros2_publisher;
    }
};

class SubprocessWithGetterOutput : public Subprocess
{
protected:

    std::string result;

    /**
     * @brief Writes childs output into variable
     * 
     * This will be started in separate thread to handle output from the child process
     * 
     * @param fd file descriptor for the pipe where stdout and stderr of child is redirected
     */
    void ChildOutputHandler(int fd)
    {
        char buffer[256];
        ssize_t n;

        while ((n = read(fd, buffer, sizeof(buffer)-1)) > 0) 
        {
            buffer[n] = '\0';
            result.append(buffer);
        }
    }

    /**
     * @brief Calls base Parent() and calls ChildOutputHandler() directly
     * 
     * @return All return values are described in this_package_root/msg/README.md
     */
    int Parent() override
    {
        int ret = Subprocess::Parent();

        ChildOutputHandler(pipes[0]);

        return ret;
    }

public:

    SubprocessWithGetterOutput(std::string exec, std::vector<std::string> args) 
    : Subprocess(exec, args) {}

    std::string get_result()
    {
        return result;
    }
};