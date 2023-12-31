#include "tello_ros2/tello_ros2.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tello_ros2::TelloROS2>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}