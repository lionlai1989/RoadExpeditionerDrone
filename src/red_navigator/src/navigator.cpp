#include <red_navigator/frontier_explorer.hpp>
#include <red_navigator/trajectory_planner.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    // Extract yaw from quaternion.
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
    // Convert yaw angle to quaternion (roll/pitch assumed zero).
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
}

double normalize_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

} // namespace

namespace red::navigator {

void save_map_image(const nav_msgs::msg::OccupancyGrid &map, int free_threshold) {
    const std::string filename = "map_debug.pgm";

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    assert(width > 0 && height > 0);
    assert(map.data.size() == static_cast<size_t>(width * height));

    std::ofstream out(filename, std::ios::binary | std::ios::out);
    assert(out.is_open());

    out << "P5\n" << width << " " << height << "\n255\n";
    std::string row;
    row.resize(static_cast<size_t>(width));
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const int idx = y * width + x;
            const int8_t value = map.data[static_cast<size_t>(idx)];
            unsigned char pixel = 205;
            if (value >= 0) {
                pixel = static_cast<unsigned char>((value > free_threshold) ? 0 : 254);
            }
            row[static_cast<size_t>(x)] = static_cast<char>(pixel);
        }
        out.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
}

void save_map_text(const nav_msgs::msg::OccupancyGrid &map) {
    const std::string filename = "map_debug.txt";

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    assert(width > 0 && height > 0);
    assert(map.data.size() == static_cast<size_t>(width * height));

    std::ofstream out(filename, std::ios::out);
    assert(out.is_open());
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            if (x > 0) {
                out << ' ';
            }
            const int idx = y * width + x;
            out << static_cast<int>(map.data[static_cast<size_t>(idx)]);
        }
        out << '\n';
    }
}

void save_map_path_image(const nav_msgs::msg::OccupancyGrid &map, int free_threshold,
                         const nav_msgs::msg::Path &path) {
    const std::string filename = "map_path_debug.pgm";

    assert(!path.header.frame_id.empty());
    assert(!map.header.frame_id.empty());
    assert(path.header.frame_id == map.header.frame_id);

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    assert(width > 0 && height > 0);
    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
    assert(map.data.size() == expected);
    const double resolution = map.info.resolution;
    assert(resolution > 0.0);

    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const double origin_yaw = quat_to_yaw(map.info.origin.orientation);
    const double cos_yaw = std::cos(origin_yaw);
    const double sin_yaw = std::sin(origin_yaw);

    std::vector<unsigned char> pixels(expected, 205);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int idx = y * width + x;
            const int8_t value = map.data[static_cast<size_t>(idx)];
            unsigned char pixel = 205;
            if (value >= 0) {
                pixel = static_cast<unsigned char>((value > free_threshold) ? 0 : 254);
            }
            pixels[static_cast<size_t>(idx)] = pixel;
        }
    }

    const unsigned char path_pixel = 0;
    auto set_pixel = [&](int gx, int gy) {
        if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
            return;
        }
        const size_t idx = static_cast<size_t>(gy * width + gx);
        pixels[idx] = path_pixel;
    };
    auto draw_line = [&](int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0);
        int dy = -std::abs(y1 - y0);
        const int sx = (x0 < x1) ? 1 : -1;
        const int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;
        while (true) {
            set_pixel(x0, y0);
            if (x0 == x1 && y0 == y1) {
                break;
            }
            const int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    };

    int prev_gx = 0;
    int prev_gy = 0;
    bool has_prev = false;
    for (const auto &pose : path.poses) {
        const double wx = pose.pose.position.x;
        const double wy = pose.pose.position.y;
        const double dx = wx - origin_x;
        const double dy = wy - origin_y;
        // Rotate into the map grid frame using the origin orientation.
        const double local_x = cos_yaw * dx + sin_yaw * dy;
        const double local_y = -sin_yaw * dx + cos_yaw * dy;
        const int gx = static_cast<int>(std::floor(local_x / resolution));
        const int gy = static_cast<int>(std::floor(local_y / resolution));
        if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
            has_prev = false;
            continue;
        }
        if (has_prev) {
            draw_line(prev_gx, prev_gy, gx, gy);
        } else {
            set_pixel(gx, gy);
        }
        prev_gx = gx;
        prev_gy = gy;
        has_prev = true;
    }

    std::ofstream out(filename, std::ios::binary | std::ios::out);
    assert(out.is_open());

    out << "P5\n" << width << " " << height << "\n255\n";
    std::string row;
    row.resize(static_cast<size_t>(width));
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            const int idx = y * width + x;
            row[static_cast<size_t>(x)] = static_cast<char>(pixels[static_cast<size_t>(idx)]);
        }
        out.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
}

} // namespace red::navigator

enum class NavState {
    LANDED,
    TAKINGOFF,
    SPINONCE,
    NAVIGATING,
    LOITER,
};

class Navigator : public rclcpp::Node {
  public:
    Navigator() : Node("navigator_node"), explorer_() {
        this->declare_parameter("hover_height", 1.5);
        this->hover_height_ = this->get_parameter("hover_height").as_double();

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.reliable();
        map_qos.transient_local();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", map_qos, std::bind(&Navigator::map_callback, this, std::placeholders::_1));

        rclcpp::QoS pose_qos(rclcpp::KeepLast(1));
        pose_qos.best_effort();
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "localization_pose", pose_qos,
            std::bind(&Navigator::pose_callback, this, std::placeholders::_1));

        this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&Navigator::odom_callback, this, std::placeholders::_1));

        this->desired_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "desired_odometry", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        this->navi_state_timer_ = rclcpp::create_timer(
            this, this->get_clock(), 200ms, std::bind(&Navigator::state_machine_step, this));

        this->explorer_timer_ = rclcpp::create_timer(this, this->get_clock(), 5s,
                                                     std::bind(&Navigator::explorer_step, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        plan_client_ =
            rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
    }

  private:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using ComputePathToPoseGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->odom_mutex_);
        this->latest_estimated_odom_ = *msg;
        if (!msg->header.frame_id.empty()) {
            this->odom_frame_id_ = msg->header.frame_id; // drone_1/odom
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(this->map_mutex_);
            this->map_ = *msg;
            this->map_ready_ = true;
            if (!msg->header.frame_id.empty()) {
                this->map_frame_id_ = msg->header.frame_id; // drone_1/map
            }
        }
        // red::navigator::save_map_image(*msg, this->explorer_.get_free_threshold());
        // red::navigator::save_map_text(*msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->pose_mutex_);
        this->current_pose_enu_ = msg->pose.pose;
        if (!msg->header.frame_id.empty()) {
            this->pose_frame_id_ = msg->header.frame_id; // drone_1/map
        }
        this->pose_ready_ = true;
    }

    void explorer_step() {

        nav_msgs::msg::OccupancyGrid map;
        geometry_msgs::msg::Pose pose;
        {
            std::lock_guard<std::mutex> lock(this->map_mutex_);
            if (!this->map_ready_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Waiting for map...");
                return;
            }
            map = this->map_;
        }
        {
            std::lock_guard<std::mutex> lock(this->pose_mutex_);
            if (!this->pose_ready_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Waiting for localization pose...");
                return;
            }
            pose = this->current_pose_enu_;
        }
        assert(map.info.resolution > 0.0);
        assert(map.info.width > 0);
        assert(map.info.height > 0);

        // only continue finding goal, path and trajectory in LOITER state.
        if (this->state_ != NavState::LOITER) {
            // RCLCPP_INFO(this->get_logger(), "Not in LOITER state. Skipping explorer step.");

            if (this->path_ready_) {
                // RCLCPP_INFO(this->get_logger(), "Saving map and path image...");
                nav_msgs::msg::Path path_snapshot;
                {
                    std::lock_guard<std::mutex> lock(this->path_mutex_);
                    path_snapshot = this->current_path_;
                }
                red::navigator::save_map_path_image(map, this->explorer_.get_free_threshold(),
                                                    path_snapshot);
                return;
            }
            return;
        }

        if (this->path_ready_) {
            // if the path is already ready, do not find new goal and path. in theory, the code
            // cannot reach here because it must be in LOITER state to reach here. but if the
            // state is loiter, then this->path_ready_ must be false.
            RCLCPP_INFO(this->get_logger(), "Path already ready. Skipping explorer step.");
            return;
        }

        red::navigator::FrontierGoal new_goal;
        if (this->explorer_.find_frontier_goal(map, pose.position.x, pose.position.y, new_goal)) {
            {
                std::lock_guard<std::mutex> lock(this->goal_mutex_);
                this->goal_x_map_ = new_goal.x_map;
                this->goal_y_map_ = new_goal.y_map;
            }
            RCLCPP_INFO(this->get_logger(), "New frontier goal: x=%.2f y=%.2f", new_goal.x_map,
                        new_goal.y_map);
            this->compute_new_path();
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No frontiers found.");
        }
    }

    void transform_goal_to_odom(const red::navigator::FrontierGoal &goal_map_frame,
                                double &goal_x_odom, double &goal_y_odom) {
        // for debugging. do not delete assert
        assert(this->map_frame_id_ == "drone_1/map");
        assert(this->odom_frame_id_ == "drone_1/odom");

        geometry_msgs::msg::PointStamped goal_map;
        goal_map.header.frame_id = this->map_frame_id_;
        goal_map.header.stamp = rclcpp::Time(0);
        goal_map.point.x = goal_map_frame.x_map;
        goal_map.point.y = goal_map_frame.y_map;
        goal_map.point.z = 0.0;

        geometry_msgs::msg::PointStamped goal_odom;
        const auto map_to_odom = this->tf_buffer_->lookupTransform(
            this->odom_frame_id_, this->map_frame_id_, rclcpp::Time(0), tf2::durationFromSec(0.2));
        tf2::doTransform(goal_map, goal_odom, map_to_odom);

        goal_x_odom = goal_odom.point.x;
        goal_y_odom = goal_odom.point.y;
    }

    void reset_path() {
        std::lock_guard<std::mutex> lock(this->path_mutex_);
        this->current_path_.poses.clear();
        this->current_path_.header.frame_id.clear();
        this->path_ready_ = false;
        this->trajectory_ready_ = false;
        this->trajectory_planner_.clear();
    }

    void compute_new_path() {
        // when entering this function, path_ready_ and trajectory_ready_ must be false. and the
        // path and trajectory are already reset.
        assert(!this->path_ready_);
        assert(!this->trajectory_ready_);

        // when entering this function, there must be a valid goal and odom.
        assert(this->latest_estimated_odom_.has_value());

        nav_msgs::msg::Odometry odom;
        red::navigator::FrontierGoal goal{};
        {
            std::lock_guard<std::mutex> lock(this->goal_mutex_);
            goal.x_map = this->goal_x_map_;
            goal.y_map = this->goal_y_map_;
        }
        {
            std::lock_guard<std::mutex> lock(this->odom_mutex_);
            odom = this->latest_estimated_odom_.value();
        }

        geometry_msgs::msg::PoseStamped start_odom;
        start_odom.header = odom.header;
        start_odom.pose = odom.pose.pose;

        // start_map's yaw can be better chosen.
        geometry_msgs::msg::PoseStamped start_map =
            this->tf_buffer_->transform(start_odom, this->map_frame_id_, tf2::durationFromSec(0.2));

        geometry_msgs::msg::PoseStamped goal_map;
        goal_map.header.frame_id = this->map_frame_id_;
        goal_map.header.stamp = start_map.header.stamp;
        goal_map.pose.position.x = goal.x_map;
        goal_map.pose.position.y = goal.y_map;
        goal_map.pose.position.z = 0.0;
        const double dx = goal_map.pose.position.x - start_map.pose.position.x;
        const double dy = goal_map.pose.position.y - start_map.pose.position.y;
        const double target_yaw = (std::hypot(dx, dy) > 1e-3)
                                      ? std::atan2(dy, dx)
                                      : quat_to_yaw(start_map.pose.orientation);
        goal_map.pose.orientation = yaw_to_quat(target_yaw);

        if (!this->plan_client_->wait_for_action_server(
                std::chrono::duration<double>(plan_timeout_sec_))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Nav2 planner action not available.");
            return;
        }

        ComputePathToPose::Goal goal_msg;
        goal_msg.start = start_map;
        goal_msg.goal = goal_map;
        goal_msg.planner_id = planner_id_;
        goal_msg.use_start = true;

        auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const ComputePathToPoseGoalHandle::WrappedResult &result) {
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Planner action failed.");
                    return;
                }
                if (result.result->path.poses.empty()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Planner returned empty path.");
                    return;
                }

                nav_msgs::msg::Path path = result.result->path;
                {
                    std::lock_guard<std::mutex> lock(this->path_mutex_);
                    this->current_path_ = path;
                    this->path_ready_ = true;
                    this->trajectory_ready_ = this->trajectory_planner_.plan(path);
                }
                if (this->path_ready_ && !this->trajectory_ready_) {
                    const std::string err_msg =
                        "Path is ready but trajectory planner failed to build a trajectory.";
                    RCLCPP_FATAL(this->get_logger(), "%s", err_msg.c_str());
                    throw std::runtime_error(err_msg);
                }
                RCLCPP_INFO(this->get_logger(),
                            "Updated trajectory with %zu samples from Nav2 path with %zu poses.",
                            this->trajectory_planner_.sample_count(), path.poses.size());
            };

        this->plan_client_->async_send_goal(goal_msg, send_goal_options);
    }

    std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Twist>
    calculate_waypoint(const nav_msgs::msg::Odometry &odom) {
        // when entering this function, the drone must be in the NAVIGATING state, which means
        // the path, trajectory and map must be ready.
        assert(this->path_ready_);
        assert(this->trajectory_ready_);

        geometry_msgs::msg::PoseStamped current_odom;
        current_odom.header = odom.header;
        current_odom.pose = odom.pose.pose;

        geometry_msgs::msg::PoseStamped current_map = this->tf_buffer_->transform(
            current_odom, this->map_frame_id_, tf2::durationFromSec(0.2));

        geometry_msgs::msg::PoseStamped target_map;
        geometry_msgs::msg::Twist trajectory_twist_map;
        bool sampled = false;
        {
            std::lock_guard<std::mutex> lock(this->path_mutex_);
            sampled = this->trajectory_planner_.sample_waypoint(current_map, target_map,
                                                                trajectory_twist_map);
        }
        if (!sampled) {
            target_map = current_map;
        }
        // Nav2 path poses can be stale; align the target pose timestamp to the current map time
        // so TF does not extrapolate into the past.
        target_map.header.frame_id = this->map_frame_id_;
        target_map.header.stamp = current_map.header.stamp;

        const auto map_to_odom =
            this->tf_buffer_->lookupTransform(this->odom_frame_id_, this->map_frame_id_,
                                              current_map.header.stamp, tf2::durationFromSec(0.2));

        geometry_msgs::msg::PoseStamped target_odom = this->tf_buffer_->transform(
            target_map, this->odom_frame_id_, tf2::durationFromSec(0.2));

        tf2::Quaternion q_map_to_odom;
        tf2::fromMsg(map_to_odom.transform.rotation, q_map_to_odom);
        const tf2::Matrix3x3 rot_map_to_odom(q_map_to_odom);
        const tf2::Vector3 velocity_map(trajectory_twist_map.linear.x,
                                        trajectory_twist_map.linear.y,
                                        trajectory_twist_map.linear.z);
        const tf2::Vector3 velocity_odom = rot_map_to_odom * velocity_map;

        // Prefer velocity-aligned yaw, then fall back to target position direction.
        const double dx = target_odom.pose.position.x - current_odom.pose.position.x;
        const double dy = target_odom.pose.position.y - current_odom.pose.position.y;
        const double speed_odom = std::hypot(velocity_odom.x(), velocity_odom.y());
        double target_yaw = quat_to_yaw(current_odom.pose.orientation);
        if (speed_odom > 1e-3) {
            target_yaw = std::atan2(velocity_odom.y(), velocity_odom.x());
        } else if (std::hypot(dx, dy) > 1e-6) {
            target_yaw = std::atan2(dy, dx);
        }

        geometry_msgs::msg::Pose desired_pose{};
        desired_pose.position = target_odom.pose.position;
        desired_pose.position.z = this->takeoff_pose_.position.z;
        desired_pose.orientation = yaw_to_quat(target_yaw);

        geometry_msgs::msg::Twist desired_twist{};
        // TODO: review this and GeometricController::compute_wrench()
        const tf2::Quaternion q_body_in_odom =
            tf2::Quaternion(current_odom.pose.orientation.x, current_odom.pose.orientation.y,
                            current_odom.pose.orientation.z, current_odom.pose.orientation.w);
        const tf2::Matrix3x3 rot_body_to_odom(q_body_in_odom);
        const tf2::Vector3 velocity_body = rot_body_to_odom.transpose() * velocity_odom;
        desired_twist.linear.x = velocity_body.x();
        desired_twist.linear.y = velocity_body.y();
        desired_twist.linear.z = velocity_body.z();
        return {desired_pose, desired_twist};
    }

    void start_spin(double current_yaw) {
        this->spin_start_yaw_ = current_yaw;
        this->spin_last_yaw_ = current_yaw;
        this->spin_accumulated_ = 0.0;
        this->spin_start_time_ = this->get_clock()->now();
        this->spin_command_yaw_ = current_yaw;
        this->spin_direction_ = 1.0;
    }

    bool update_spin_progress(double current_yaw) {
        const double delta = normalize_angle(current_yaw - this->spin_last_yaw_);
        if (this->spin_direction_ * delta > 0.0) {
            this->spin_accumulated_ += delta;
        }
        this->spin_last_yaw_ = current_yaw;
        return std::abs(this->spin_accumulated_) >= kTwoPi;
    }

    void state_machine_step() {
        /**
         * There are 5 states:
         * LANDED (0): initial state. The drone is landed on the ground. Immediately switch to
         * TAKINGOFF state.
         *
         * TAKINGOFF (1): take off to hover_height. If the current position to the target
         * position is less than takeoff_tolerance, switch to SPINONCE.
         *
         * SPINONCE (2): rotate 360 degrees at the current position. then switch to LOITER.
         *
         * NAVIGATING (3): fly toward the current goal setpoint. if reached the goal, switch to
         * LOITER.
         *
         * LOITER (4): hover at the hold position. find goal and path. if found, switch to
         * NAVIGATING.
         *
         * So, the state transition should be:
         * 0 -> 1 -> 2 -> 4 -> 3 -> 2 -> 4 -> 3 -> 2 -> 4 -> 3 -> 2 -> 4 -> ...
         */

        // get current position
        std::optional<nav_msgs::msg::Odometry> current_odom;
        {
            std::lock_guard<std::mutex> lock(this->odom_mutex_);
            current_odom = this->latest_estimated_odom_;
        }
        if (!current_odom.has_value()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Waiting for valid odom");
            return;
        }

        // Only check if the goal is reached when the path and trajectory are ready and in
        // NAVIGATING state.
        if (this->path_ready_ && this->trajectory_ready_ && this->state_ == NavState::NAVIGATING) {
            std::lock_guard<std::mutex> lock(this->goal_mutex_);
            double goal_x_odom = 0.0;
            double goal_y_odom = 0.0;
            red::navigator::FrontierGoal goal;
            goal.x_map = this->goal_x_map_;
            goal.y_map = this->goal_y_map_;
            this->transform_goal_to_odom(goal, goal_x_odom, goal_y_odom);
            const double dx = goal_x_odom - current_odom->pose.pose.position.x;
            const double dy = goal_y_odom - current_odom->pose.pose.position.y;
            if (std::hypot(dx, dy) < this->goal_reached_radius_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached. Resetting path.");
                this->reset_path();
            }
        }

        NavState prev_state = this->state_;

        geometry_msgs::msg::Pose desired_pose{};
        geometry_msgs::msg::Twist desired_twist{};
        switch (this->state_) {
        case NavState::LANDED: {
            this->takeoff_pose_.position.x = current_odom->pose.pose.position.x;
            this->takeoff_pose_.position.y = current_odom->pose.pose.position.y;
            this->takeoff_pose_.position.z =
                current_odom->pose.pose.position.z + this->hover_height_;
            this->takeoff_pose_.orientation =
                yaw_to_quat(quat_to_yaw(current_odom->pose.pose.orientation));

            // Transit to TAKINGOFF
            this->state_ = NavState::TAKINGOFF;
            break;
        }
        case NavState::TAKINGOFF: {
            const float dx = current_odom->pose.pose.position.x - this->takeoff_pose_.position.x;
            const float dy = current_odom->pose.pose.position.y - this->takeoff_pose_.position.y;
            const float dz = current_odom->pose.pose.position.z - this->takeoff_pose_.position.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < this->takeoff_tolerance_) {
                this->hold_x_ = current_odom->pose.pose.position.x;
                this->hold_y_ = current_odom->pose.pose.position.y;
                this->hold_yaw_ = quat_to_yaw(current_odom->pose.pose.orientation);
                this->start_spin(this->hold_yaw_);

                // Transit to SPINONCE
                this->state_ = NavState::SPINONCE;
            } else {
                desired_pose.position = this->takeoff_pose_.position;
                desired_pose.orientation = this->takeoff_pose_.orientation;
            }
            break;
        }
        case NavState::SPINONCE: {
            if (this->update_spin_progress(quat_to_yaw(current_odom->pose.pose.orientation))) {
                this->hold_x_ = current_odom->pose.pose.position.x;
                this->hold_y_ = current_odom->pose.pose.position.y;
                this->hold_yaw_ = quat_to_yaw(current_odom->pose.pose.orientation);

                // Transit to LOITER
                this->state_ = NavState::LOITER;
            } else {
                desired_pose.position.x = this->hold_x_;
                desired_pose.position.y = this->hold_y_;
                desired_pose.position.z = this->takeoff_pose_.position.z;
                const rclcpp::Time now = this->get_clock()->now();
                const double elapsed = (now - this->spin_start_time_).seconds();
                this->spin_command_yaw_ = this->spin_start_yaw_ +
                                          this->spin_direction_ * this->spin_rate_rad_s_ * elapsed;
                desired_pose.orientation = yaw_to_quat(this->spin_command_yaw_);
                desired_twist.angular.z = this->spin_direction_ * this->spin_rate_rad_s_;
            }
            break;
        }
        case NavState::NAVIGATING: {
            if (this->path_ready_ && this->trajectory_ready_) {
                std::tie(desired_pose, desired_twist) = this->calculate_waypoint(*current_odom);
            } else {
                this->hold_x_ = current_odom->pose.pose.position.x;
                this->hold_y_ = current_odom->pose.pose.position.y;
                this->hold_yaw_ = quat_to_yaw(current_odom->pose.pose.orientation);
                this->start_spin(this->hold_yaw_);

                // Transit to SPINONCE
                this->state_ = NavState::SPINONCE;
            }
            break;
        }
        case NavState::LOITER: {
            if (this->path_ready_ && this->trajectory_ready_) {
                this->state_ = NavState::NAVIGATING;
            } else {
                desired_pose.position.x = this->hold_x_;
                desired_pose.position.y = this->hold_y_;
                desired_pose.position.z = this->takeoff_pose_.position.z;
                desired_pose.orientation = yaw_to_quat(this->hold_yaw_);
            }
            break;
        }
        } // switch this->state_

        // When state changed, do not publish desired odometry because desired odometry is
        // invalid.
        if (prev_state != this->state_) {
            RCLCPP_INFO(this->get_logger(), "State changed: %d -> %d", static_cast<int>(prev_state),
                        static_cast<int>(this->state_));

            return;
        }

        // A null quaternion (0,0,0,0) is invalid and typically indicates an uninitialized
        // command.
        const auto &q = desired_pose.orientation;
        const double q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        assert(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
               std::isfinite(q.w) && std::isfinite(q_norm_sq) && q_norm_sq > 1e-12);

        nav_msgs::msg::Odometry desired_odom;
        desired_odom.header = current_odom->header;
        desired_odom.pose.pose = desired_pose;
        desired_odom.twist.twist = desired_twist;
        {
            std::lock_guard<std::mutex> lock(this->odom_mutex_);
            this->latest_desired_odom_ = desired_odom;
        }
        this->desired_odom_pub_->publish(desired_odom);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr desired_odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::TimerBase::SharedPtr navi_state_timer_;
    rclcpp::TimerBase::SharedPtr explorer_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::Client<ComputePathToPose>::SharedPtr plan_client_;

    std::mutex path_mutex_;
    nav_msgs::msg::Path current_path_;
    bool path_ready_{false};

    bool trajectory_ready_{false};

    std::string planner_id_{"GridBased"}; // must match planner_plugins in nav2_params.yaml
    double plan_timeout_sec_{2.0};

    std::mutex odom_mutex_;
    std::optional<nav_msgs::msg::Odometry> latest_estimated_odom_;
    std::optional<nav_msgs::msg::Odometry> latest_desired_odom_;
    std::string odom_frame_id_;

    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid map_;
    bool map_ready_{false};
    std::string map_frame_id_;

    std::mutex pose_mutex_;
    geometry_msgs::msg::Pose current_pose_enu_;
    bool pose_ready_{false};
    std::string pose_frame_id_;

    double hover_height_{0.0};
    float takeoff_tolerance_{0.2f};
    geometry_msgs::msg::Pose takeoff_pose_{};
    std::mutex goal_mutex_;
    // Goal position in the map frame.
    double goal_x_map_{0.0};
    double goal_y_map_{0.0};
    double goal_reached_radius_{0.1};

    double hold_x_{0.0};
    double hold_y_{0.0};
    double hold_yaw_{0.0};
    double spin_start_yaw_{0.0};
    double spin_last_yaw_{0.0};
    double spin_accumulated_{0.0};
    double spin_command_yaw_{0.0};
    rclcpp::Time spin_start_time_{};
    double spin_rate_rad_s_{0.3};
    double spin_direction_{1.0};

    NavState state_{NavState::LANDED};

    red::navigator::FrontierExplorer explorer_;
    red::navigator::TrajectoryPlanner trajectory_planner_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Navigator>());
    rclcpp::shutdown();
    return 0;
}
