#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomTfBroadcaster : public rclcpp::Node {
  public:
    OdomTfBroadcaster()
        : Node("odom_tf_broadcaster"),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {
        odom_frame_id_ = declare_parameter<std::string>("odom_frame_id");
        base_frame_id_ = declare_parameter<std::string>("base_frame_id");

        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", rclcpp::QoS(50),
            std::bind(&OdomTfBroadcaster::handle_odom, this, std::placeholders::_1));
    }

  private:
    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // msg->header.frame_id: drone_1/odom
        // msg->child_frame_id: base_link

        // Always overrides frame_id and child_frame_id.
        const std::string parent_frame = odom_frame_id_;
        const std::string child_frame = base_frame_id_;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    std::string odom_frame_id_;
    std::string base_frame_id_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
