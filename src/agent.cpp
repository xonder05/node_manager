#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include "std_msgs/msg/string.hpp"
#include "node_manager/msg/node_red_command.hpp"

#include <memory>
#include <thread>

#include "node_manager/subprocess_manager.hpp"

class NodeManager : public rclcpp::Node
{

private:

    SubprocessManager manager;

    rclcpp::Subscription<node_manager::msg::NodeRedCommand>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    int id = 1;

    void callback(node_manager::msg::NodeRedCommand::UniquePtr msg) 
    {
        if (msg->manager_id != id) return;

        switch (msg->message_type)
        {
            case 0:
            {
                std::cout << "starting a node" << std::endl;
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = this->create_publisher<std_msgs::msg::String>("/management/stdout/" + msg->node_id, 5);
                manager.StartNode(msg->node_id, msg->package_name, msg->node_name, publisher);
                break;
            }
            
            case 1:
            {
                manager.StopNode(msg->node_name);
                break;
            }

            default:
                RCLCPP_INFO(this->get_logger(), "I heard unknown message type\n");
                break;
        }

    }

public: 

    NodeManager() : Node("node_manager")
    {
        auto qos = rclcpp::SystemDefaultsQoS()
        .keep_last(1) // history
        .reliable() // reliability
        .transient_local(); // durability

        publisher_ = this->create_publisher<std_msgs::msg::String>
            ("/management/agent_advertisement", qos);

        std_msgs::msg::String msg;
        msg.data = "this is my id";
        this->publisher_->publish(msg);


        subscription_ = this->create_subscription<node_manager::msg::NodeRedCommand>("/management/commands", 10, std::bind(&NodeManager::callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
                std::chrono::seconds(10),
                [this]() {
                    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
                });

        std::cout << "hello from init" << std::endl;

        RCLCPP_INFO(this->get_logger(), "NodeManager initialization done.");
    }

    ~NodeManager() {

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NodeManager>());
    rclcpp::shutdown();   
    
    std::cout << "returning from main normally" << std::endl;
    return 0;
}