/**
 * D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 2011
 * IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, pp. 2520-2525,
 * doi: 10.1109/ICRA.2011.5980409.
 */

#pragma once

#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/path.hpp>

#include <string>
#include <vector>

namespace red::navigator {

class TrajectoryPlanner {
  public:
    struct Limits {
        double max_speed_mps{0.8};
        double max_accel_mps2{0.8};
        double lookahead_time_s{0.6};
        double min_lookahead_m{0.15};
    };

    TrajectoryPlanner();
    explicit TrajectoryPlanner(const Limits &limits);

    void set_limits(const Limits &limits);
    const Limits &limits() const;

    bool plan(const nav_msgs::msg::Path &path_map);
    void clear();

    bool is_ready() const;
    double total_length() const;
    size_t sample_count() const;
    const std::string &frame_id() const;

    bool sample_waypoint(const geometry_msgs::msg::PoseStamped &current_pose_map,
                         geometry_msgs::msg::PoseStamped &waypoint_map,
                         geometry_msgs::msg::Twist &waypoint_twist_map) const;

  private:
    static constexpr double kEpsilon{1e-6};
    static constexpr int kPolynomialOrder{7};
    static constexpr int kCoefficientCount{kPolynomialOrder + 1};
    static constexpr size_t kMaxSegmentsPerSolve{40};

    struct SamplePoint {
        double time_s{0.0};
        double arc_length_m{0.0};
        std::array<double, 3> position{};
        std::array<double, 3> velocity{};
    };

    static double min_segment_time(double distance_m, const Limits &limits);
    static std::array<double, kCoefficientCount> polynomial_basis(double t, int derivative_order);

    bool solve_axis_minimum_snap(
        const std::vector<double> &waypoint_values,
        std::vector<std::array<double, kCoefficientCount>> &segment_coeffs) const;
    bool solve_subpath(const std::vector<geometry_msgs::msg::PoseStamped> &subpath_points_map,
                       std::vector<double> &subpath_segment_times_s,
                       std::vector<std::array<double, kCoefficientCount>> &subpath_coeffs_x,
                       std::vector<std::array<double, kCoefficientCount>> &subpath_coeffs_y,
                       std::vector<std::array<double, kCoefficientCount>> &subpath_coeffs_z) const;
    size_t time_segment_index(double time_s) const;
    double evaluate_axis(const std::vector<std::array<double, kCoefficientCount>> &coeffs,
                         size_t segment_idx, double local_time_s, int derivative_order) const;
    void evaluate_state(double time_s, std::array<double, 3> &position,
                        std::array<double, 3> &velocity) const;
    void build_samples();
    double project_time_from_xy(double x, double y, double *arc_length_m) const;
    double speed_at_time(double time_s) const;
    double time_at_arc(double arc_length_m) const;
    bool heading_direction_at_time(double time_s, double &dir_x, double &dir_y) const;

    Limits limits_{};
    std::vector<geometry_msgs::msg::PoseStamped> path_points_map_;
    std::vector<double> segment_times_s_;
    std::vector<double> cumulative_times_s_;
    std::vector<std::array<double, kCoefficientCount>> coeffs_x_;
    std::vector<std::array<double, kCoefficientCount>> coeffs_y_;
    std::vector<std::array<double, kCoefficientCount>> coeffs_z_;
    std::vector<SamplePoint> samples_;
    double total_time_s_{0.0};
    double total_length_m_{0.0};
    std::string frame_id_;
};

} // namespace red::navigator
