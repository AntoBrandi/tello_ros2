#include "tello_ros2/tello_ros2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "cv_bridge/cv_bridge.h"

using namespace std::placeholders;

namespace tello_ros2
{
TelloROS2::TelloROS2(const rclcpp::NodeOptions & options) 
    : Node("tello_ros2", options)
{
}

void TelloROS2::init()
{
    tello_ptr_ = std::make_shared<Tello>();
    if (!tello_ptr_->Bind())
    {
        RCLCPP_ERROR(get_logger(), "Cannot connect to Tello");
    }

    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());

    // ROS 2 Interface
    takeoff_srv_ = create_service<std_srvs::srv::Empty>("tello/takeoff", std::bind(&TelloROS2::takeoffCallback, this, _1, _2, _3));
    land_srv_ = create_service<std_srvs::srv::Empty>("tello/land", std::bind(&TelloROS2::landCallback, this, _1, _2, _3));
    flip_r_srv_ = create_service<std_srvs::srv::Empty>("tello/flip_r", std::bind(&TelloROS2::flipRCallback, this, _1, _2, _3));
    flip_l_srv_ = create_service<std_srvs::srv::Empty>("tello/flip_l", std::bind(&TelloROS2::flipLCallback, this, _1, _2, _3));
    flip_f_srv_ = create_service<std_srvs::srv::Empty>("tello/flip_f", std::bind(&TelloROS2::flipFCallback, this, _1, _2, _3));
    flip_b_srv_ = create_service<std_srvs::srv::Empty>("tello/flip_b", std::bind(&TelloROS2::flipBCallback, this, _1, _2, _3));
    hover_srv_ = create_service<std_srvs::srv::Empty>("tello/hover", std::bind(&TelloROS2::hoverCallback, this, _1, _2, _3));
    emergency_srv_ = create_service<std_srvs::srv::Empty>("tello/emergency", std::bind(&TelloROS2::emergencyCallback, this, _1, _2, _3));
    enable_stream_srv_ = create_service<std_srvs::srv::Empty>("tello/enable_stream", std::bind(&TelloROS2::enableStreamCallback, this, _1, _2, _3));
    disable_stream_srv_ = create_service<std_srvs::srv::Empty>("tello/disable_stream", std::bind(&TelloROS2::disableStreamCallback, this, _1, _2, _3));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("tello/cmd_vel", 10, std::bind(&TelloROS2::cmdVelCallback, this, _1));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("tello/imu", 10);
    battery_pub_ = create_publisher<std_msgs::msg::Float32>("tello/battery", 10);
    temperature_pub_ = create_publisher<std_msgs::msg::Float32>("tello/temperature", 10);
    height_pub_ = create_publisher<std_msgs::msg::Float32>("tello/height", 10);
    barometer_pub_ = create_publisher<std_msgs::msg::Float32>("tello/barometer", 10);
    camera_pub_ = it_->advertise("tello/camera", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TelloROS2::timerCallback, this));
    camera_timer_ = create_wall_timer(std::chrono::microseconds(5), std::bind(&TelloROS2::cameraLoop, this));

    enableStream();
}

bool TelloROS2::takeoffCallback(const std::shared_ptr<rmw_request_id_t>,
                         const std_srvs::srv::Empty::Request::SharedPtr,
                         std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("takeoff");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::landCallback(const std::shared_ptr<rmw_request_id_t>,
                  const std_srvs::srv::Empty::Request::SharedPtr,
                  std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("land");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::flipRCallback(const std::shared_ptr<rmw_request_id_t>,
                   const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("flip r");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::flipLCallback(const std::shared_ptr<rmw_request_id_t>,
                   const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("flip l");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::flipFCallback(const std::shared_ptr<rmw_request_id_t>,
                   const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("flip f");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::flipBCallback(const std::shared_ptr<rmw_request_id_t>,
                   const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("flip b");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::hoverCallback(const std::shared_ptr<rmw_request_id_t>,
                   const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("stop");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::emergencyCallback(const std::shared_ptr<rmw_request_id_t>,
                       const std_srvs::srv::Empty::Request::SharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("emergency");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}

bool TelloROS2::enableStreamCallback(const std::shared_ptr<rmw_request_id_t>,
                          const std_srvs::srv::Empty::Request::SharedPtr,
                          std_srvs::srv::Empty::Response::SharedPtr)
{
    enableStream();
    return true;
}

bool TelloROS2::disableStreamCallback(const std::shared_ptr<rmw_request_id_t>,
                           const std_srvs::srv::Empty::Request::SharedPtr,
                           std_srvs::srv::Empty::Response::SharedPtr)
{
    tello_ptr_->SendCommand("streamoff");
    while (!(tello_ptr_->ReceiveResponse()));
    tello_ptr_->CloseStream();
    camera_timer_.reset();
    return true;
}

void TelloROS2::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
   std::string cmd = "rc ";
    
    int yaw_velocity = std::max(-100, std::min(100, (int)msg->angular.z));
    int forward_backward_velocity = std::max(-100, std::min(100, (int)msg->linear.x));
    int left_right_velocity = std::max(-100, std::min(100, (int)msg->linear.y));
    int up_down_velocity = std::max(-100, std::min(100, (int)msg->linear.z));

    cmd += std::to_string(left_right_velocity) + " " +
           std::to_string(forward_backward_velocity) + " " +
           std::to_string(up_down_velocity) + " " +
           std::to_string(yaw_velocity);

    tello_ptr_->SendCommand(cmd); 
}

void TelloROS2::timerCallback()
{
    // Create and Publish the IMU message
    std::map<std::string,std::string> state;
    tello_ptr_->GetState(state);
    if(state.find("roll") != state.end() &&
       state.find("pitch") != state.end() &&
       state.find("yaw") != state.end() &&
       state.find("agx") != state.end() &&
       state.find("agy") != state.end() &&
       state.find("agz") != state.end() &&
       state.find("vgx") != state.end() &&
       state.find("vgy") != state.end() &&
       state.find("vgz") != state.end())
    {
        tf2::Quaternion orientation;
        orientation.setRPY(std::stod(state["roll"]), 
                        std::stod(state["pitch"]),
                        std::stod(state["yaw"]));
        sensor_msgs::msg::Imu imu;
        imu.header.frame_id = "imu";
        imu.linear_acceleration.x = std::stod(state["agx"])/100;
        imu.linear_acceleration.y = std::stod(state["agy"])/100;
        imu.linear_acceleration.z = std::stod(state["agz"])/100;
        imu.angular_velocity.x = std::stod(state["vgx"]);
        imu.angular_velocity.y = std::stod(state["vgy"]);
        imu.angular_velocity.z = std::stod(state["vgz"]);
        imu.orientation.x = orientation.getX();
        imu.orientation.y = orientation.getY();
        imu.orientation.z = orientation.getZ();
        imu.orientation.w = orientation.getW();
        imu.header.stamp = get_clock()->now();
        imu_pub_->publish(imu);
    }
    

    // Create and publish Battery
    if(state.find("bat") != state.end())
    {
        std_msgs::msg::Float32 battery;
        battery.data = std::stod(state["bat"]);
        battery_pub_->publish(battery);
    }

    // Create and publish Temperature
    if(state.find("templ") != state.end() && 
       state.find("temph") != state.end())
    {
        std_msgs::msg::Float32 temperature;
        temperature.data = (std::stod(state["templ"]) + std::stod(state["temph"])) / 2;
        temperature_pub_->publish(temperature);
    }

    // Create and publish Height
    if(state.find("h") != state.end())
    {
        std_msgs::msg::Float32 height;
        height.data = std::stod(state["h"]) / 100;
        height_pub_->publish(height);
    }

    // Create and publish Barometer
    if(state.find("baro") != state.end())
    {
        std_msgs::msg::Float32 barometer;
        barometer.data = std::stod(state["baro"]) / 100;
        barometer_pub_->publish(barometer); 
    }
}

void TelloROS2::cameraLoop()
{
    cv::Mat frame;
    tello_ptr_->GetFrame(frame);
    if(frame.empty())
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Empty Frame");
    }
    if(!frame.empty())
    {
        sensor_msgs::msg::Image::SharedPtr camera_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        camera_pub_.publish(camera_msg);
    }
}

void TelloROS2::enableStream()
{
    tello_ptr_->SendCommand("streamon");
    while (!(tello_ptr_->ReceiveResponse()));
    tello_ptr_->OpenStream();
    camera_timer_ = create_wall_timer(std::chrono::microseconds(100), std::bind(&TelloROS2::cameraLoop, this));
}
}  // namespace tello_ros2
