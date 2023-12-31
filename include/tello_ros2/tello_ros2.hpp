#ifndef TELLO_ROS2_H
#define TELLO_ROS2_H

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tello_ros2/tello.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace tello_ros2
{
class TelloROS2 : public rclcpp::Node
{
public:
    TelloROS2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void init();

private:
    std::shared_ptr<Tello> tello_ptr_;

    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher camera_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr takeoff_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr land_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr flip_r_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr flip_l_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr flip_b_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr flip_f_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hover_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr emergency_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_stream_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_stream_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr barometer_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr camera_timer_;

    bool takeoffCallback(const std::shared_ptr<rmw_request_id_t>,
                         const std_srvs::srv::Empty::Request::SharedPtr,
                         std_srvs::srv::Empty::Response::SharedPtr);

    bool landCallback(const std::shared_ptr<rmw_request_id_t>,
                      const std_srvs::srv::Empty::Request::SharedPtr,
                      std_srvs::srv::Empty::Response::SharedPtr);

    bool flipRCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr);

    bool flipLCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr);

    bool flipFCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr);

    bool flipBCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr);

    bool hoverCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr);

    bool emergencyCallback(const std::shared_ptr<rmw_request_id_t>,
                           const std_srvs::srv::Empty::Request::SharedPtr,
                           std_srvs::srv::Empty::Response::SharedPtr);

    bool enableStreamCallback(const std::shared_ptr<rmw_request_id_t>,
                              const std_srvs::srv::Empty::Request::SharedPtr,
                              std_srvs::srv::Empty::Response::SharedPtr);

    bool disableStreamCallback(const std::shared_ptr<rmw_request_id_t>,
                               const std_srvs::srv::Empty::Request::SharedPtr,
                               std_srvs::srv::Empty::Response::SharedPtr);

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void timerCallback();

    void cameraLoop();

    void enableStream();
};
}  // namespace tello_ros2
#endif //TELLO_ROS2_H
