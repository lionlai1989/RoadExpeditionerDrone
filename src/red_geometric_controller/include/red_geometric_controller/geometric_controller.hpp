#pragma once

#include <array>
#include <mutex>
#include <optional>
#include <utility>

#include <Eigen/Dense>
#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "red_geometric_controller/quadcopter_params.hpp"

namespace red::geometric_controller {

class GeometricController : public rclcpp::Node {
  public:
    GeometricController();

    std::array<double, 4>
    compute_motor_speeds(const geometry_msgs::msg::Pose &curr_pose,
                         const geometry_msgs::msg::Twist &curr_twist,
                         const geometry_msgs::msg::Pose &desired_pose,
                         const geometry_msgs::msg::Twist &desired_twist) const;

    std::pair<double, Eigen::Vector3d>
    compute_wrench(const geometry_msgs::msg::Pose &curr_pose,
                   const geometry_msgs::msg::Twist &curr_twist,
                   const geometry_msgs::msg::Pose &desired_pose,
                   const geometry_msgs::msg::Twist &desired_twist) const;

  private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void desired_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_step();

    std::pair<Eigen::Matrix3d, Eigen::Vector3d>
    compute_desired_orientation(const Eigen::Vector3d &acc, double yaw) const;

    double kp_position_{1.0};
    double kv_linvel_{3.0};
    Eigen::Vector3d kr_rotmat_{4.0, 4.0, 0.1};
    Eigen::Vector3d kw_angvel_{2.0, 2.0, 0.05};

    QuadcopterParams drone_params_;

    std::mutex odom_mutex_;
    std::optional<nav_msgs::msg::Odometry> latest_estimated_odom_;
    std::optional<nav_msgs::msg::Odometry> latest_desired_odom_;

    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr desired_odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace red::geometric_controller
