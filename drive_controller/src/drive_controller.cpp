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
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include "robot_msgs/msg/u_int8_vector.hpp"
#include "robot_msgs/msg/float32_vector.hpp"
#include "robot_msgs/msg/omega_angles.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class DriveController : public rclcpp::Node {
private:
    const size_t GLOBAL_VELS_NUM_ = 3;
    const size_t GLOBAL_POSES_NUM_ = 3;

    rclcpp::Publisher<robot_msgs::msg::UInt8Vector>::SharedPtr serial_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<robot_msgs::msg::OmegaAngles>::SharedPtr camera_sub_;
    rclcpp::Subscription<robot_msgs::msg::OmegaAngles>::SharedPtr gripper_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr task_sub_;

    size_t cmd_size_, pose_num_, pose_size_, vel_num_, vel_size_;
    size_t angles_size_, camera_yaw_idx_, camera_pitch_idx_, gripper_yaw_idx_, gripper_pitch_idx_, task_idx_;

    uint8_t camera_yaw_, camera_pitch_, gripper_yaw_, gripper_pitch_, task_;
    geometry_msgs::msg::Twist msg_;

public:
    DriveController();

private:
    std::vector<float> calcForwardKinematics(std::vector<float> V);    
    void cmdVelCallback(const geometry_msgs::msg::Twist& msg);
    void cameraCallback(const robot_msgs::msg::OmegaAngles& msg);
    void gripperCallback(const robot_msgs::msg::OmegaAngles& msg);
    void taskCallback(const std_msgs::msg::UInt8& msg);
};


DriveController::DriveController() : Node("drive_controller") {
    task_ = 0;

    this->declare_parameter("cmd_vel_sub", "");
    this->declare_parameter("camera_sub", "");
    this->declare_parameter("gripper_sub", "");
    this->declare_parameter("serial_sub", "");
    this->declare_parameter("task_sub", "");
    this->declare_parameter("cmd_size", 0);
    this->declare_parameter("vel_num", 0);
    this->declare_parameter("vel_size", 0);
    this->declare_parameter("angles_size", 0);
    this->declare_parameter("camera_yaw", 0);
    this->declare_parameter("camera_pitch", 0);
    this->declare_parameter("gripper_yaw", 0);
    this->declare_parameter("gripper_pitch", 0);
    this->declare_parameter("task", 0);

    std::string cmd_vel_sub_topic = this->get_parameter("cmd_vel_sub").as_string();
    std::string camera_sub_topic = this->get_parameter("camera_sub").as_string();
    std::string gripper_sub_topic = this->get_parameter("gripper_sub").as_string();
    std::string serial_sub_topic = this->get_parameter("serial_sub").as_string();
    std::string task_sub_topic = this->get_parameter("task_sub").as_string();
    cmd_size_ = this->get_parameter("cmd_size").as_int();
    vel_num_ = this->get_parameter("vel_num").as_int();
    vel_size_ = this->get_parameter("vel_size").as_int();
    angles_size_ = this->get_parameter("angles_size").as_int();
    camera_yaw_idx_ = this->get_parameter("camera_yaw").as_int();
    camera_pitch_idx_ = this->get_parameter("camera_pitch").as_int();
    gripper_yaw_idx_ = this->get_parameter("gripper_yaw").as_int();
    gripper_pitch_idx_ = this->get_parameter("gripper_pitch").as_int();
    task_idx_ = this->get_parameter("task").as_int();

    RCLCPP_INFO(this->get_logger(), "vel_sub_topic: '%s'", cmd_vel_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_sub_topic: '%s'", camera_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "gripper_sub_topic: '%s'", gripper_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_sub_topic: '%s'", serial_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "task_sub_topic: '%s'", task_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_size: %ld", cmd_size_);
    RCLCPP_INFO(this->get_logger(), "vel_num: %ld", vel_num_);
    RCLCPP_INFO(this->get_logger(), "vel_size: %ld", vel_size_);
    RCLCPP_INFO(this->get_logger(), "angles_size: %ld", angles_size_);
    RCLCPP_INFO(this->get_logger(), "camera_yaw: %ld", camera_yaw_idx_);
    RCLCPP_INFO(this->get_logger(), "camera_pitch: %ld", camera_pitch_idx_);
    RCLCPP_INFO(this->get_logger(), "gripper_yaw: %ld", gripper_yaw_idx_);
    RCLCPP_INFO(this->get_logger(), "gripper_pitch: %ld", gripper_pitch_idx_);
    RCLCPP_INFO(this->get_logger(), "task: %ld", task_idx_);

    serial_pub_ = this->create_publisher<robot_msgs::msg::UInt8Vector>(serial_sub_topic, 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_sub_topic, 10, std::bind(&DriveController::cmdVelCallback, this, _1));
    camera_sub_ = this->create_subscription<robot_msgs::msg::OmegaAngles>(camera_sub_topic, 10, std::bind(&DriveController::cameraCallback, this, _1));
    gripper_sub_ = this->create_subscription<robot_msgs::msg::OmegaAngles>(gripper_sub_topic, 10, std::bind(&DriveController::gripperCallback, this, _1));
    task_sub_ = this->create_subscription<std_msgs::msg::UInt8>(task_sub_topic, 10, std::bind(&DriveController::taskCallback, this, _1));
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

    RCLCPP_INFO(this->get_logger(), "%d %d %d %d, %d", camera_yaw_, camera_pitch_, gripper_yaw_, gripper_pitch_, task_);

    std::memcpy(serial_msg.data.data() + camera_yaw_idx_, &camera_yaw_, angles_size_);
    std::memcpy(serial_msg.data.data() + camera_pitch_idx_, &camera_pitch_, angles_size_);
    std::memcpy(serial_msg.data.data() + gripper_yaw_idx_, &gripper_yaw_, angles_size_);
    std::memcpy(serial_msg.data.data() + gripper_pitch_idx_, &gripper_pitch_, angles_size_);
    std::memcpy(serial_msg.data.data() + task_idx_, &task_, sizeof(uint8_t));

    serial_pub_->publish(serial_msg);
}


void DriveController::cameraCallback(const robot_msgs::msg::OmegaAngles& msg) {
    camera_yaw_ = msg.horiz_angle;
    camera_pitch_ = msg.vert_angle;
    cmdVelCallback(msg_);
}


void DriveController::gripperCallback(const robot_msgs::msg::OmegaAngles& msg) {
    gripper_yaw_ = msg.horiz_angle;
    gripper_pitch_ = msg.vert_angle;
    cmdVelCallback(msg_);
}


void DriveController::taskCallback(const std_msgs::msg::UInt8& msg) {
    task_ = msg.data;
    cmdVelCallback(msg_);
    task_ = 0;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>()); 
    rclcpp::shutdown();

    return 0;
}
