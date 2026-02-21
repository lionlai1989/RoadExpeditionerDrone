/**
Adapt either groundtruth odometry or OpenVINS odometry into a stable `odometry` + `odom->base_link`
interface for the rest of the system.

Startup behavior: TODO: verify if this is correct?
1) Bootstrap from Gazebo groundtruth:
   T_odom_base = inverse(T_world_base0) * T_world_base
2) Once OpenVINS starts publishing valid odometry, switch permanently to OpenVINS.
3) Freeze one alignment transform at switch time to keep outputs continuous:
   T_odom_ovGlobal = T_odom_base_atSwitch * inverse(T_ovGlobal_imu_atSwitch)
4) After switch:
   T_odom_base = T_odom_ovGlobal * T_ovGlobal_imu

We assume IMU and base_link are colocated in this simulation model.

The first received groundtruth pose is used as the odom-frame origin:
  T_odom_base = inverse(T_world_base0) * T_world_base
This makes odometry start at (0, 0, 0) and identity orientation at initialization.
The normalized odometry is published on `odometry`, and the same pose is broadcast as TF
`odom -> base_link`.

- **`world`**: Gazebo's absolute coordinate system. Ground truth is naturally relative to this.
- **`map`**: RTAB-Map's globally consistent frame, which acts as the top-level fixed frame in the
  real world ( correcting for odometry drift).
- **`odom`**: A local, continuous frame. By convention, this starts at `(0, 0, 0)` when the robot's
  VIO or Odometry system initializes, regardless of where the robot powers on.

When using `groundtruth`, the Gazebo bridge outputs the absolute pose (`world -> base_link`). By
taking that absolute pose and directly naming its frame `odom`, the system is forcing `odom` to be
identical to the absolute `world` pose.
Conversely, OpenVINS follows standard conventions: it initializes at `(0, 0, 0)` in its local `odom`
frame, completely ignorant of its absolute `world` start location.

**The Danger**: If downstream nodes (like `red_navigator` or `geometric_controller`) were to
subscribe to `/odometry` assuming it gives them their absolute world location, they would function
perfectly when `vio_source == groundtruth`. However, the moment you switch to `openvins`,
those nodes will suddenly receive `(0, 0, 0)` for their position, causing catastrophic behavior
(e.g. attempting to violently fly 10 meters away to reach an absolute waypoint).

To make the transition strictly "plug-and-play", we must ensure that `groundtruth` mimics `openvins`
perfectly. This means `groundtruth` odometry MUST also start at `(0, 0, 0)` in the `odom` frame.

This forces all downstream modules to use proper `tf2` lookups against the `map` frame for global
navigation goals, rather than cheating by reading the absolute `/odometry` topic.
 */

#include <cassert>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
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

        groundtruth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "groundtruth_odometry", rclcpp::QoS(50),
            std::bind(&OdomAdapter::handle_groundtruth_odom, this, std::placeholders::_1));
        openvins_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "odomimu", rclcpp::QoS(50),
            std::bind(&OdomAdapter::handle_openvins_odom, this, std::placeholders::_1));
        odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(50));
    }

  private:
    static tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose &pose) {
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                          pose.orientation.w);
        q.normalize();
        const tf2::Vector3 t(pose.position.x, pose.position.y, pose.position.z);
        return tf2::Transform(q, t);
    }

    void publish_odom_and_tf(const nav_msgs::msg::Odometry &source_msg,
                             const tf2::Transform &odom_to_base) {
        const tf2::Vector3 p = odom_to_base.getOrigin();
        const tf2::Quaternion q = odom_to_base.getRotation();

        nav_msgs::msg::Odometry odom_msg = source_msg;
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

    void handle_groundtruth_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (switched_to_openvins_) {
            return;
        }

        const tf2::Transform world_to_base = pose_to_transform(msg->pose.pose);
        if (!origin_initialized_) {
            world_to_base0_inverse_ = world_to_base.inverse();
            origin_initialized_ = true;
        }

        const tf2::Transform odom_to_base = world_to_base0_inverse_ * world_to_base;
        publish_odom_and_tf(*msg, odom_to_base);
    }

    std::string odom_frame_id_;
    std::string base_frame_id_;

    bool origin_initialized_ = false;
    tf2::Transform world_to_base0_inverse_;

    bool switched_to_openvins_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr groundtruth_subscription_;
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
