#include "node_manager/node_manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<NodeManager> node_manager = std::make_shared<NodeManager>();

    rclcpp::spin(node_manager);
    rclcpp::shutdown();
    return 0;
}