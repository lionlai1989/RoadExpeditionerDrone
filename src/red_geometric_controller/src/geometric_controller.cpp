/**
 * A geometric controller node that computes and publishes motor speeds at 100 Hz.
 * subscribe: odometry and desired_odometry
 * publish: command/motor_speed
 */

#include "red_geometric_controller/geometric_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "red_geometric_controller/motor_mixing.hpp"
#include "red_geometric_controller/transform.hpp"

namespace red::geometric_controller {
namespace {

Eigen::Vector3d rotation_error(const Eigen::Matrix3d &rot_current,
                               const Eigen::Matrix3d &rot_desired) {
    const Eigen::Matrix3d rot_err =
        rot_desired.transpose() * rot_current - rot_current.transpose() * rot_desired;
    return Eigen::Vector3d(0.5 * (rot_err(2, 1) - rot_err(1, 2)),
                           0.5 * (rot_err(0, 2) - rot_err(2, 0)),
                           0.5 * (rot_err(1, 0) - rot_err(0, 1)));
}

} // namespace

std::array<double, 4>
GeometricController::compute_motor_speeds(const geometry_msgs::msg::Pose &curr_pose,
                                          const geometry_msgs::msg::Twist &curr_twist,
                                          const geometry_msgs::msg::Pose &desired_pose,
                                          const geometry_msgs::msg::Twist &desired_twist) const {
    const auto [force, torque] = compute_wrench(curr_pose, curr_twist, desired_pose, desired_twist);
    return wrench_to_motor_speeds(force, torque, drone_params_.rotor_positions,
                                  drone_params_.rotor_cf, drone_params_.rotor_cd,
                                  drone_params_.yaw_signs, drone_params_.motor_max_rot_velocity);
}

std::pair<double, Eigen::Vector3d>
GeometricController::compute_wrench(const geometry_msgs::msg::Pose &curr_pose,
                                    const geometry_msgs::msg::Twist &curr_twist,
                                    const geometry_msgs::msg::Pose &desired_pose,
                                    const geometry_msgs::msg::Twist &desired_twist) const {
    const Eigen::Vector3d curr_pos(curr_pose.position.x, curr_pose.position.y,
                                   curr_pose.position.z);
    const Eigen::Matrix3d curr_rot =
        quat_to_rotmat(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y,
                       curr_pose.orientation.z);

    const Eigen::Vector3d curr_linvel_body(curr_twist.linear.x, curr_twist.linear.y,
                                           curr_twist.linear.z);
    const Eigen::Vector3d curr_linvel = curr_rot * curr_linvel_body;
    const Eigen::Vector3d curr_angvel(curr_twist.angular.x, curr_twist.angular.y,
                                      curr_twist.angular.z);

    const Eigen::Vector3d des_pos(desired_pose.position.x, desired_pose.position.y,
                                  desired_pose.position.z);
    const Eigen::Matrix3d des_rot =
        quat_to_rotmat(desired_pose.orientation.w, desired_pose.orientation.x,
                       desired_pose.orientation.y, desired_pose.orientation.z);
    const auto des_euler = quat_to_euler(desired_pose.orientation.w, desired_pose.orientation.x,
                                         desired_pose.orientation.y, desired_pose.orientation.z);
    const double des_yaw = std::get<2>(des_euler);

    const Eigen::Vector3d des_linvel_body(desired_twist.linear.x, desired_twist.linear.y,
                                          desired_twist.linear.z);
    const Eigen::Vector3d des_lin_vel = des_rot * des_linvel_body;
    const Eigen::Vector3d des_angvel(desired_twist.angular.x, desired_twist.angular.y,
                                     desired_twist.angular.z);

    const Eigen::Vector3d e_pos = curr_pos - des_pos;
    const Eigen::Vector3d e_linvel = curr_linvel - des_lin_vel;

    const Eigen::Vector3d acc_cmd =
        -kp_position_ * e_pos - kv_linvel_ * e_linvel + kGravity * Eigen::Vector3d(0.0, 0.0, 1.0);
    const auto [commanded_rot, acc_cmd_limited] = compute_desired_orientation(acc_cmd, des_yaw);
    double force = drone_params_.mass * acc_cmd_limited.dot(curr_rot.col(2));
    force =
        std::clamp(force, drone_params_.force_z_limit.first, drone_params_.force_z_limit.second);
    const Eigen::Vector3d e_rot = rotation_error(curr_rot, commanded_rot);
    const Eigen::Vector3d e_angvel =
        curr_angvel - curr_rot.transpose() * (commanded_rot * des_angvel);
    const Eigen::Vector3d torque =
        -(kr_rotmat_.asDiagonal() * e_rot) - (kw_angvel_.asDiagonal() * e_angvel) +
        curr_angvel.cross(drone_params_.inertia.cwiseProduct(curr_angvel));

    return {force, torque};
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d>
GeometricController::compute_desired_orientation(const Eigen::Vector3d &acc, double yaw) const {
    Eigen::Vector3d a = acc;
    if (a.z() < 1e-6) {
        a.z() = 1e-6;
    }
    const double max_accel = drone_params_.max_accel;
    if (a.z() > max_accel) {
        a.z() = max_accel;
    }
    const double horiz = std::hypot(a.x(), a.y());
    const double max_horiz_by_thrust =
        std::sqrt(std::max(0.0, max_accel * max_accel - a.z() * a.z()));
    const double max_horiz_by_tilt = std::tan(drone_params_.max_tilt_angle) * std::abs(a.z());
    const double max_horiz = std::min(max_horiz_by_thrust, max_horiz_by_tilt);
    if (horiz > max_horiz) {
        const double scale = max_horiz / (horiz + 1e-9);
        a.x() *= scale;
        a.y() *= scale;
    }

    const double norm_a = a.norm();
    Eigen::Vector3d z_w_des(0.0, 0.0, 1.0);
    if (norm_a > 1e-6) {
        z_w_des = a / norm_a;
    }

    Eigen::Vector3d x_c_des(std::cos(yaw), std::sin(yaw), 0.0);
    if (std::abs(z_w_des.dot(x_c_des)) > 0.999) {
        x_c_des = Eigen::Vector3d(std::cos(yaw + 0.01), std::sin(yaw + 0.01), 0.0);
    }

    Eigen::Vector3d y_w_des = z_w_des.cross(x_c_des);
    y_w_des.normalize();
    Eigen::Vector3d x_w_des = y_w_des.cross(z_w_des);
    x_w_des.normalize();

    Eigen::Matrix3d rot;
    rot.col(0) = x_w_des;
    rot.col(1) = y_w_des;
    rot.col(2) = z_w_des;
    return {rot, a};
}

GeometricController::GeometricController() : rclcpp::Node("geometric_controller_node") {
    this->motor_cmd_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
        "command/motor_speed", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
    odom_qos.best_effort();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", odom_qos,
        std::bind(&GeometricController::odom_callback, this, std::placeholders::_1));
    desired_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "desired_odometry", odom_qos,
        std::bind(&GeometricController::desired_odom_callback, this, std::placeholders::_1));

    control_timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(10),
                                          std::bind(&GeometricController::control_step, this));
}

void GeometricController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_estimated_odom_ = *msg;
}

void GeometricController::desired_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_desired_odom_ = *msg;
}

void GeometricController::control_step() {
    std::optional<nav_msgs::msg::Odometry> estimated_odom;
    std::optional<nav_msgs::msg::Odometry> desired_odom;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        estimated_odom = latest_estimated_odom_;
        desired_odom = latest_desired_odom_;
    }
    if (!estimated_odom.has_value() || !desired_odom.has_value()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for odometry and desired odometry");
        return;
    }

    const std::array<double, 4> motor_speeds =
        compute_motor_speeds(estimated_odom->pose.pose, estimated_odom->twist.twist,
                             desired_odom->pose.pose, desired_odom->twist.twist);
    actuator_msgs::msg::Actuators motor_cmd;
    motor_cmd.velocity.assign(motor_speeds.begin(), motor_speeds.end());
    motor_cmd_pub_->publish(motor_cmd);
}
} // namespace red::geometric_controller

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<red::geometric_controller::GeometricController>());
    rclcpp::shutdown();
    return 0;
}
