/**
 * `odom_adapter` provides a unified, system-facing odometry interface:
 *   - Topic: `/<drone_id>/odometry`
 *   - TF: `/<drone_id>/odom -> /<drone_id>/base_link`
 *
 * In `openvins` mode, it bootstraps from Gazebo groundtruth odometry. After odomimu callback is
 * triggered, it transitions to OpenVINS odometry immediately.
 */

#include <cassert>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomAdapter : public rclcpp::Node {
  public:
    OdomAdapter()
        : Node("odom_adapter"),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {
        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id");
        vio_source_ = declare_parameter<std::string>("vio_source");
        assert((vio_source_ == "groundtruth") || (vio_source_ == "synthetic") ||
               (vio_source_ == "openvins"));

        if (vio_source_ == "openvins") {
            openvins_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "odomimu", rclcpp::QoS(50),
                std::bind(&OdomAdapter::handle_openvins_odom, this, std::placeholders::_1));
            groundtruth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "groundtruth_odometry", rclcpp::QoS(50),
                std::bind(&OdomAdapter::handle_groundtruth_odom, this, std::placeholders::_1));
        } else if (vio_source_ == "synthetic") {
            synthetic_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "groundtruth_odometry", rclcpp::QoS(50),
                std::bind(&OdomAdapter::handle_synthetic_odom, this, std::placeholders::_1));
        } else if (vio_source_ == "groundtruth") {
            groundtruth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "groundtruth_odometry", rclcpp::QoS(50),
                std::bind(&OdomAdapter::handle_groundtruth_odom, this, std::placeholders::_1));
        } else {
            throw std::runtime_error("Invalid vio_source: " + vio_source_);
        }

        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(50));
    }

  private:
    static bool is_finite(const double value) { return std::isfinite(value); }

    static bool is_pose_finite(const geometry_msgs::msg::Pose &pose) {
        return is_finite(pose.position.x) && is_finite(pose.position.y) &&
               is_finite(pose.position.z) && is_finite(pose.orientation.x) &&
               is_finite(pose.orientation.y) && is_finite(pose.orientation.z) &&
               is_finite(pose.orientation.w);
    }

    static double position_step_m(const geometry_msgs::msg::Pose &a,
                                  const geometry_msgs::msg::Pose &b) {
        const double dx = a.position.x - b.position.x;
        const double dy = a.position.y - b.position.y;
        const double dz = a.position.z - b.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose &pose) {
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                          pose.orientation.w);
        const double length2 = q.length2();
        if (!is_finite(length2) || length2 < 1e-12) {
            q = tf2::Quaternion::getIdentity();
        } else {
            q.normalize();
        }
        const tf2::Vector3 t(pose.position.x, pose.position.y, pose.position.z);
        return tf2::Transform(q, t);
    }

    void publish_odom_and_tf(nav_msgs::msg::Odometry odom_msg, const tf2::Transform &odom_to_base) {
        const tf2::Vector3 p = odom_to_base.getOrigin();
        const tf2::Quaternion q = odom_to_base.getRotation();

        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        odom_msg.pose.pose.position.x = p.x();
        odom_msg.pose.pose.position.y = p.y();
        odom_msg.pose.pose.position.z = p.z();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_publisher_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = odom_msg.header.stamp;
        transform.header.frame_id = odom_frame_id_;
        transform.child_frame_id = base_frame_id_;
        transform.transform.translation.x = odom_msg.pose.pose.position.x;
        transform.transform.translation.y = odom_msg.pose.pose.position.y;
        transform.transform.translation.z = odom_msg.pose.pose.position.z;
        transform.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transform);
    }

    void handle_synthetic_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // TODO: add noise and random walk to xyz only.
        const tf2::Transform world_to_base = pose_to_transform(msg->pose.pose);
        if (!groundtruth_origin_initialized_) {
            world_to_base0_inverse_ = world_to_base.inverse();
            groundtruth_origin_initialized_ = true;
        }
        const tf2::Transform groundtruth_odom_to_base = world_to_base0_inverse_ * world_to_base;
        publish_odom_and_tf(*msg, groundtruth_odom_to_base);
    }

    void handle_groundtruth_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (openvins_origin_initialized_) {
            return;
        }

        const tf2::Transform world_to_base = pose_to_transform(msg->pose.pose);
        if (!groundtruth_origin_initialized_) {
            world_to_base0_inverse_ = world_to_base.inverse();
            groundtruth_origin_initialized_ = true;
        }

        const tf2::Transform groundtruth_odom_to_base = world_to_base0_inverse_ * world_to_base;

        publish_odom_and_tf(*msg, groundtruth_odom_to_base);
    }

    void handle_openvins_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        assert(is_pose_finite(msg->pose.pose));

        const tf2::Transform ov_global_to_imu = pose_to_transform(msg->pose.pose);
        if (!openvins_origin_initialized_) {
            openvins_origin_initialized_ = true;
            RCLCPP_INFO(get_logger(), "\x1b[1;36mSwitched odometry source to OpenVINS\x1b[0m");
        }

        const tf2::Transform odom_to_base = ov_global_to_imu;
        publish_odom_and_tf(*msg, odom_to_base);
    }

    std::string odom_frame_id_;
    std::string base_frame_id_;

    std::string vio_source_;

    bool groundtruth_origin_initialized_ = false;
    tf2::Transform world_to_base0_inverse_;

    bool openvins_origin_initialized_ = false;
    tf2::Transform odom_to_openvins_global_;

    geometry_msgs::msg::Pose prev_openvins_pose_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr groundtruth_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr synthetic_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr openvins_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomAdapter>());
    rclcpp::shutdown();
    return 0;
}
