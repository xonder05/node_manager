#include "rclcpp/rclcpp.hpp"

#include <rclcpp/qos.hpp>

#include "std_msgs/msg/string.hpp"

#include "interfaces/msg/node_red_command.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <thread>

#include "node_manager/subprocess_manager.hpp"
// #include "websocket.cpp"

class NodeManager : public rclcpp::Node
{

private:

    SubprocessManager manager;

    rclcpp::Subscription<interfaces::msg::NodeRedCommand>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // boost::asio::io_context ioc_;
    // std::shared_ptr<listener> ws_listener_;
    // std::thread ws_thread_;
    rclcpp::TimerBase::SharedPtr timer_;

    int id = 1;

    void callback(interfaces::msg::NodeRedCommand::UniquePtr msg) 
    {
        if (msg->manager_id != id) return;

        switch (msg->message_type)
        {
            case 0:
            {
                std::cout << "starting a node" << std::endl;
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher = this->create_publisher<std_msgs::msg::String>("/management/stdout/" + msg->node_name + std::to_string(1), 5);
                manager.StartNode(msg->package_name, msg->node_name, publisher);
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
    // , ioc_(1)
    {

        // auto endpoint = boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("0.0.0.0"), 8080);
        // ws_listener_ = std::make_shared<listener>(ioc_, endpoint);
        // ws_listener_->run();

        // ws_thread_ = std::thread([this]() { ioc_.run(); });

        
        auto qos = rclcpp::SystemDefaultsQoS()
        .keep_last(1)                  // History = KEEP_LAST, depth = 1
        .reliable()                    // Reliability = RELIABLE
        .transient_local();            // Durability = TRANSIENT_LOCAL

        publisher_ = this->create_publisher<std_msgs::msg::String>
            ("/management/agent_advertisement", qos);

        std_msgs::msg::String msg;
        msg.data = "this is my id";
        this->publisher_->publish(msg);


        subscription_ = this->create_subscription<interfaces::msg::NodeRedCommand>("/management/commands", 10, std::bind(&NodeManager::callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
                std::chrono::seconds(10),
                [this]() {
                    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
                });

        std::cout << "hello from init" << std::endl;

        RCLCPP_INFO(this->get_logger(), "NodeManager initialization done.");
    }

    ~NodeManager() {
        // ioc_.stop();
        // if (ws_thread_.joinable())
        //     ws_thread_.join();
    }

};

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<NodeManager>());
        std::cout << "we are screwed?" << std::endl;
        rclcpp::shutdown();   
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "still alive after shutdown" << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;
}