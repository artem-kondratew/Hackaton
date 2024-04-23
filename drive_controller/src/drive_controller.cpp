#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_msgs/msg/u_int8_vector.hpp"
#include "robot_msgs/msg/float32_vector.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class DriveController : public rclcpp::Node {
private:
    const size_t GLOBAL_VELS_NUM_ = 3;
    const size_t GLOBAL_POSES_NUM_ = 3;

    rclcpp::Publisher<robot_msgs::msg::UInt8Vector>::SharedPtr serial_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    size_t cmd_size_, pose_num_, pose_size_, vel_num_, vel_size_;

public:
    DriveController();

private:
    std::vector<float> calcForwardKinematics(std::vector<float> V);    
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg);
};


DriveController::DriveController() : Node("drive_controller") {
    this->declare_parameter("cmd_vel_sub", "");
    this->declare_parameter("serial_sub", "");
    this->declare_parameter("cmd_size", 0);
    this->declare_parameter("vel_num", 0);
    this->declare_parameter("vel_size", 0);

    std::string cmd_vel_sub_topic = this->get_parameter("cmd_vel_sub").as_string();
    std::string serial_sub_topic = this->get_parameter("serial_sub").as_string();
    cmd_size_ = this->get_parameter("cmd_size").as_int();
    vel_num_ = this->get_parameter("vel_num").as_int();
    vel_size_ = this->get_parameter("vel_size").as_int();

    RCLCPP_INFO(this->get_logger(), "vel_sub_topic: '%s'", cmd_vel_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_sub_topic: '%s'", serial_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_size: %ld", cmd_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", vel_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", vel_size_);

    serial_pub_ = this->create_publisher<robot_msgs::msg::UInt8Vector>(serial_sub_topic, 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_sub_topic, 10, std::bind(&DriveController::cmdVelCallback, this, _1));
}


std::vector<float> DriveController::calcForwardKinematics(std::vector<float> V) {
    if (V.size() != GLOBAL_VELS_NUM_) {
        RCLCPP_FATAL(this->get_logger(), "Wrong cmd_vel_size");
        rclcpp::shutdown();
    }

    float vx = V[0];
    float wz = V[2];

    std::vector<float> W;
    W.resize(vel_num_);

    W[0] = vx - wz;
    W[1] = vx + wz;

    RCLCPP_INFO(this->get_logger(), "%f, %f", W[0], W[1]);

    return W;
}


void DriveController::cmdVelCallback(const geometry_msgs::msg::Twist& msg) {
    auto serial_msg = robot_msgs::msg::UInt8Vector();
    serial_msg.data.resize(cmd_size_);

    std::vector<float> V;
    V.push_back(msg.linear.x);
    V.push_back(msg.linear.y);
    V.push_back(msg.angular.z);

    std::vector<float> W = calcForwardKinematics(V);

    for (size_t i = 0; i < W.size(); i++) {
        std::memcpy(serial_msg.data.data() + i * vel_size_, W.data() + i, vel_size_);
    }

    serial_pub_->publish(serial_msg);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>()); 
    rclcpp::shutdown();

    return 0;
}
