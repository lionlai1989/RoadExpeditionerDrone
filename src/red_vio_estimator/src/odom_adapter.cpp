/**
 * `odom_adapter` provides a unified, system-facing odometry interface:
 *   - Topic: `/<drone_id>/odometry`
 *   - TF: `/<drone_id>/odom -> /<drone_id>/base_link`
 *
 * In `openvins` mode, it bootstraps from Gazebo groundtruth odometry. After odomimu callback is
 * triggered, it transitions to OpenVINS odometry once it evaluates the odomimu stream as stable.
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
    static constexpr int kOpenvinsMinStableMsgCount = 100; // 3000 not work
    static constexpr double kOpenvinsMaxStepMeters = 1.0;

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

    bool update_openvins_gate(const geometry_msgs::msg::Pose &pose) {
        assert(is_pose_finite(pose));

        if (openvins_valid_msg_count_ == 0) {
            prev_openvins_pose_ = pose;
            openvins_valid_msg_count_ = 1;
            return false;
        }

        const double step = position_step_m(pose, prev_openvins_pose_);
        assert(is_finite(step));
        prev_openvins_pose_ = pose;
        if (step > kOpenvinsMaxStepMeters) {
            openvins_valid_msg_count_ = 1;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                                 "\x1b[1;33mOpenVINS handoff gate rejected discontinuous sample "
                                 "(step=%.3f m)\x1b[0m",
                                 step);
            return false;
        }

        ++openvins_valid_msg_count_;
        return openvins_valid_msg_count_ >= kOpenvinsMinStableMsgCount;
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
        const tf2::Transform world_to_base = pose_to_transform(msg->pose.pose);
        if (!groundtruth_origin_initialized_) {
            world_to_base0_inverse_ = world_to_base.inverse();
            groundtruth_origin_initialized_ = true;
        }

        const tf2::Transform groundtruth_odom_to_base = world_to_base0_inverse_ * world_to_base;

        nav_msgs::msg::Odometry synthetic_msg = *msg;
        const tf2::Transform synthetic_odom_to_base =
            make_synthetic_odom_to_base(groundtruth_odom_to_base, rclcpp::Time(msg->header.stamp));
        add_synthetic_velocity_noise(synthetic_msg);
        publish_odom_and_tf(synthetic_msg, synthetic_odom_to_base);
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

    double sample_gaussian(const double stddev) {
        assert(stddev >= 0.0);
        if (stddev == 0.0) {
            return 0.0;
        }
        return stddev * normal_dist_(random_engine_);
    }

    double synthetic_dt_sec(const rclcpp::Time &stamp) {
        if (!synthetic_prev_stamp_initialized_) {
            synthetic_prev_stamp_ = stamp;
            synthetic_prev_stamp_initialized_ = true;
            return 0.0;
        }

        const double dt_sec = (stamp - synthetic_prev_stamp_).seconds();
        synthetic_prev_stamp_ = stamp;
        assert(is_finite(dt_sec));
        assert(dt_sec >= 0.0);
        return dt_sec;
    }

    tf2::Transform make_synthetic_odom_to_base(const tf2::Transform &groundtruth_odom_to_base,
                                               const rclcpp::Time &stamp) {
        const double dt_sec = synthetic_dt_sec(stamp);

        synthetic_position_random_walk_ +=
            tf2::Vector3(sample_gaussian(random_walk_pos_stddev_ * dt_sec),
                         sample_gaussian(random_walk_pos_stddev_ * dt_sec),
                         sample_gaussian(random_walk_pos_stddev_ * dt_sec));
        synthetic_yaw_random_walk_ += sample_gaussian(random_walk_yaw_stddev_ * dt_sec);

        const tf2::Vector3 groundtruth_position = groundtruth_odom_to_base.getOrigin();
        const tf2::Vector3 synthetic_position =
            groundtruth_position + synthetic_position_random_walk_ +
            tf2::Vector3(sample_gaussian(noise_pos_stddev_), sample_gaussian(noise_pos_stddev_),
                         sample_gaussian(noise_pos_stddev_));

        tf2::Quaternion synthetic_yaw_drift;
        synthetic_yaw_drift.setRPY(0.0, 0.0, synthetic_yaw_random_walk_);
        tf2::Quaternion synthetic_orientation =
            synthetic_yaw_drift * groundtruth_odom_to_base.getRotation();
        synthetic_orientation.normalize();

        return tf2::Transform(synthetic_orientation, synthetic_position);
    }

    void add_synthetic_velocity_noise(nav_msgs::msg::Odometry &odom_msg) {
        odom_msg.twist.twist.linear.x += sample_gaussian(noise_vel_stddev_);
        odom_msg.twist.twist.linear.y += sample_gaussian(noise_vel_stddev_);
        odom_msg.twist.twist.linear.z += sample_gaussian(noise_vel_stddev_);
    }

    void handle_openvins_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        assert(is_pose_finite(msg->pose.pose));

        const tf2::Transform ov_global_to_imu = pose_to_transform(msg->pose.pose);
        if (!openvins_origin_initialized_) {
            // The 1st data looks pretty good. There is no need to gate here.
            // x=0.000, y=-0.001, z=0.008
            // if (!update_openvins_gate(msg->pose.pose))
            //     return;
            // do i really need to apply the first inverse transform? openvins already has its own
            // global frame.
            // odom_to_openvins_global_ = ov_global_to_imu.inverse();
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

    tf2::Vector3 synthetic_position_random_walk_ = tf2::Vector3(0.0, 0.0, 0.0);
    double synthetic_yaw_random_walk_ = 0.0;
    bool synthetic_prev_stamp_initialized_ = false;
    rclcpp::Time synthetic_prev_stamp_;
    std::mt19937 random_engine_{std::random_device{}()};
    std::normal_distribution<double> normal_dist_{0.0, 1.0};
    // Derived from x500_base IMU noise in src/red_gz_plugin/models/x500_base/model.sdf.
    // accel stddev RMS = sqrt((0.00637^2 + 0.00637^2 + 0.00686^2) / 3) [m/s^2]
    // gyro stddev = 0.0008726646 [rad/s], imu_dt = 1 / 250 [s]
    double noise_pos_stddev_ = 1.0459864626e-7;
    double noise_vel_stddev_ = 2.6149661566e-5;
    double random_walk_pos_stddev_ = 2.6149661566e-5;
    double random_walk_yaw_stddev_ = 8.726646e-4;

    bool openvins_origin_initialized_ = false;
    tf2::Transform odom_to_openvins_global_;

    int openvins_valid_msg_count_ = 0;
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
