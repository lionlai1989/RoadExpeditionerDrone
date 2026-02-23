#include "red_navigator/trajectory_planner.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace red::navigator {

namespace {

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
}

double point_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

TrajectoryPlanner::TrajectoryPlanner() { this->set_limits(this->limits_); }

TrajectoryPlanner::TrajectoryPlanner(const Limits &limits) { this->set_limits(limits); }

void TrajectoryPlanner::set_limits(const Limits &limits) {
    this->limits_.max_speed_mps = std::max(limits.max_speed_mps, 0.05);
    this->limits_.max_accel_mps2 = std::max(limits.max_accel_mps2, 0.05);
    this->limits_.lookahead_time_s = std::max(limits.lookahead_time_s, 0.01);
    this->limits_.min_lookahead_m = std::max(limits.min_lookahead_m, 0.01);
}

const TrajectoryPlanner::Limits &TrajectoryPlanner::limits() const { return this->limits_; }

double TrajectoryPlanner::min_segment_time(double distance_m, const Limits &limits) {
    const double distance = std::max(distance_m, 0.0);
    const double max_speed = std::max(limits.max_speed_mps, 0.05);
    const double max_accel = std::max(limits.max_accel_mps2, 0.05);
    const double accel_time = max_speed / max_accel;
    const double accel_distance = 0.5 * max_accel * accel_time * accel_time;

    double min_time = 0.0;
    if (distance < 2.0 * accel_distance) {
        min_time = 2.0 * std::sqrt(distance / max_accel);
    } else {
        const double cruise_distance = distance - 2.0 * accel_distance;
        min_time = 2.0 * accel_time + cruise_distance / max_speed;
    }
    return std::max(0.1, min_time * 1.15);
}

std::array<double, TrajectoryPlanner::kCoefficientCount>
TrajectoryPlanner::polynomial_basis(double t, int derivative_order) {
    std::array<double, kCoefficientCount> row{};
    for (int power = derivative_order; power < kCoefficientCount; ++power) {
        double coefficient = 1.0;
        for (int d = 0; d < derivative_order; ++d) {
            coefficient *= static_cast<double>(power - d);
        }
        row[power] = coefficient * std::pow(t, static_cast<double>(power - derivative_order));
    }
    return row;
}

void TrajectoryPlanner::clear() {
    this->path_points_map_.clear();
    this->segment_times_s_.clear();
    this->cumulative_times_s_.clear();
    this->coeffs_x_.clear();
    this->coeffs_y_.clear();
    this->coeffs_z_.clear();
    this->samples_.clear();
    this->total_time_s_ = 0.0;
    this->total_length_m_ = 0.0;
    this->frame_id_.clear();
}

bool TrajectoryPlanner::plan(const nav_msgs::msg::Path &path_map) {
    this->clear();
    if (path_map.poses.empty()) {
        return false;
    }

    this->frame_id_ = path_map.header.frame_id;
    if (this->frame_id_.empty()) {
        this->frame_id_ = path_map.poses.front().header.frame_id;
    }
    if (this->frame_id_.empty()) {
        return false;
    }

    this->path_points_map_.reserve(path_map.poses.size());
    for (const auto &pose_in : path_map.poses) {
        auto pose = pose_in;
        pose.header.frame_id = this->frame_id_;
        if (this->path_points_map_.empty()) {
            this->path_points_map_.push_back(pose);
            continue;
        }

        const auto &prev = this->path_points_map_.back().pose.position;
        const auto &curr = pose.pose.position;
        if (point_distance(prev, curr) <= kEpsilon) {
            continue;
        }
        this->path_points_map_.push_back(pose);
    }
    if (this->path_points_map_.empty()) {
        return false;
    }

    if (this->path_points_map_.size() == 1U) {
        this->build_samples();
        return this->is_ready();
    }

    const size_t total_segment_count = this->path_points_map_.size() - 1U;
    this->segment_times_s_.clear();
    this->coeffs_x_.clear();
    this->coeffs_y_.clear();
    this->coeffs_z_.clear();
    this->segment_times_s_.reserve(total_segment_count);
    this->coeffs_x_.reserve(total_segment_count);
    this->coeffs_y_.reserve(total_segment_count);
    this->coeffs_z_.reserve(total_segment_count);

    size_t chunk_start_waypoint_idx = 0U;
    while (chunk_start_waypoint_idx + 1U < this->path_points_map_.size()) {
        const size_t remaining_segments =
            (this->path_points_map_.size() - 1U) - chunk_start_waypoint_idx;
        const size_t chunk_segment_count = std::min(kMaxSegmentsPerSolve, remaining_segments);
        const size_t chunk_end_waypoint_idx = chunk_start_waypoint_idx + chunk_segment_count;

        std::vector<geometry_msgs::msg::PoseStamped> subpath_points_map;
        subpath_points_map.reserve(chunk_segment_count + 1U);
        for (size_t i = chunk_start_waypoint_idx; i <= chunk_end_waypoint_idx; ++i) {
            subpath_points_map.push_back(this->path_points_map_[i]);
        }

        std::vector<double> subpath_segment_times_s;
        std::vector<std::array<double, kCoefficientCount>> subpath_coeffs_x;
        std::vector<std::array<double, kCoefficientCount>> subpath_coeffs_y;
        std::vector<std::array<double, kCoefficientCount>> subpath_coeffs_z;
        if (!this->solve_subpath(subpath_points_map, subpath_segment_times_s, subpath_coeffs_x,
                                 subpath_coeffs_y, subpath_coeffs_z)) {
            this->clear();
            return false;
        }
        if (subpath_segment_times_s.size() != chunk_segment_count ||
            subpath_coeffs_x.size() != chunk_segment_count ||
            subpath_coeffs_y.size() != chunk_segment_count ||
            subpath_coeffs_z.size() != chunk_segment_count) {
            this->clear();
            return false;
        }

        this->segment_times_s_.insert(this->segment_times_s_.end(), subpath_segment_times_s.begin(),
                                      subpath_segment_times_s.end());
        this->coeffs_x_.insert(this->coeffs_x_.end(), subpath_coeffs_x.begin(),
                               subpath_coeffs_x.end());
        this->coeffs_y_.insert(this->coeffs_y_.end(), subpath_coeffs_y.begin(),
                               subpath_coeffs_y.end());
        this->coeffs_z_.insert(this->coeffs_z_.end(), subpath_coeffs_z.begin(),
                               subpath_coeffs_z.end());

        chunk_start_waypoint_idx = chunk_end_waypoint_idx;
    }

    this->cumulative_times_s_.assign(this->segment_times_s_.size() + 1U, 0.0);
    for (size_t i = 0; i < this->segment_times_s_.size(); ++i) {
        this->cumulative_times_s_[i + 1U] =
            this->cumulative_times_s_[i] + this->segment_times_s_[i];
    }
    this->total_time_s_ = this->cumulative_times_s_.back();

    this->build_samples();

    return this->is_ready();
}

bool TrajectoryPlanner::solve_subpath(
    const std::vector<geometry_msgs::msg::PoseStamped> &subpath_points_map,
    std::vector<double> &subpath_segment_times_s,
    std::vector<std::array<double, kCoefficientCount>> &subpath_coeffs_x,
    std::vector<std::array<double, kCoefficientCount>> &subpath_coeffs_y,
    std::vector<std::array<double, kCoefficientCount>> &subpath_coeffs_z) const {
    subpath_segment_times_s.clear();
    subpath_coeffs_x.clear();
    subpath_coeffs_y.clear();
    subpath_coeffs_z.clear();
    if (subpath_points_map.size() < 2U) {
        return false;
    }

    const size_t segment_count = subpath_points_map.size() - 1U;
    subpath_segment_times_s.assign(segment_count, 0.0);
    for (size_t i = 0; i < segment_count; ++i) {
        const auto &a = subpath_points_map[i].pose.position;
        const auto &b = subpath_points_map[i + 1U].pose.position;
        const double segment_distance = point_distance(a, b);
        subpath_segment_times_s[i] = min_segment_time(segment_distance, this->limits_);
    }

    std::vector<double> waypoint_x(subpath_points_map.size(), 0.0);
    std::vector<double> waypoint_y(subpath_points_map.size(), 0.0);
    std::vector<double> waypoint_z(subpath_points_map.size(), 0.0);
    for (size_t i = 0; i < subpath_points_map.size(); ++i) {
        waypoint_x[i] = subpath_points_map[i].pose.position.x;
        waypoint_y[i] = subpath_points_map[i].pose.position.y;
        waypoint_z[i] = subpath_points_map[i].pose.position.z;
    }

    TrajectoryPlanner subpath_planner(this->limits_);
    subpath_planner.segment_times_s_ = subpath_segment_times_s;
    const bool solved_x = subpath_planner.solve_axis_minimum_snap(waypoint_x, subpath_coeffs_x);
    const bool solved_y = subpath_planner.solve_axis_minimum_snap(waypoint_y, subpath_coeffs_y);
    const bool solved_z = subpath_planner.solve_axis_minimum_snap(waypoint_z, subpath_coeffs_z);
    return solved_x && solved_y && solved_z;
}

bool TrajectoryPlanner::is_ready() const {
    if (this->path_points_map_.empty() || this->frame_id_.empty() || this->samples_.empty()) {
        return false;
    }
    if (this->path_points_map_.size() == 1U) {
        return true;
    }
    return !this->segment_times_s_.empty() &&
           this->coeffs_x_.size() == this->segment_times_s_.size() &&
           this->coeffs_y_.size() == this->segment_times_s_.size() &&
           this->coeffs_z_.size() == this->segment_times_s_.size();
}

double TrajectoryPlanner::total_length() const { return this->total_length_m_; }

size_t TrajectoryPlanner::sample_count() const { return this->samples_.size(); }

const std::string &TrajectoryPlanner::frame_id() const { return this->frame_id_; }

bool TrajectoryPlanner::solve_axis_minimum_snap(
    const std::vector<double> &waypoint_values,
    std::vector<std::array<double, kCoefficientCount>> &segment_coeffs) const {
    segment_coeffs.clear();
    const size_t segment_count = this->segment_times_s_.size();
    if (segment_count == 0U) {
        return waypoint_values.size() <= 1U;
    }
    if (waypoint_values.size() != segment_count + 1U) {
        return false;
    }

    const int variable_count = static_cast<int>(segment_count * kCoefficientCount);
    const int continuity_constraints =
        static_cast<int>((segment_count > 1U) ? 3U * (segment_count - 1U) : 0U);
    const int constraint_count = static_cast<int>(2U * segment_count) + 4 + continuity_constraints;

    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(variable_count, variable_count);
    for (size_t segment_idx = 0; segment_idx < segment_count; ++segment_idx) {
        const double duration_s = this->segment_times_s_[segment_idx];
        const int base = static_cast<int>(segment_idx * kCoefficientCount);
        for (int m = 4; m < kCoefficientCount; ++m) {
            const double m_factor = static_cast<double>(m * (m - 1) * (m - 2) * (m - 3));
            for (int n = 4; n < kCoefficientCount; ++n) {
                const double n_factor = static_cast<double>(n * (n - 1) * (n - 2) * (n - 3));
                const double exponent = static_cast<double>(m + n - 7);
                q(base + m, base + n) =
                    m_factor * n_factor * std::pow(duration_s, exponent) / exponent;
            }
        }
    }
    q.diagonal().array() += 1e-9;

    Eigen::MatrixXd aeq = Eigen::MatrixXd::Zero(constraint_count, variable_count);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(constraint_count);
    int row = 0;

    for (size_t segment_idx = 0; segment_idx < segment_count; ++segment_idx) {
        const int base = static_cast<int>(segment_idx * kCoefficientCount);
        const auto start_basis = polynomial_basis(0.0, 0);
        const auto end_basis = polynomial_basis(this->segment_times_s_[segment_idx], 0);

        for (int c = 0; c < kCoefficientCount; ++c) {
            aeq(row, base + c) = start_basis[c];
        }
        beq(row) = waypoint_values[segment_idx];
        ++row;

        for (int c = 0; c < kCoefficientCount; ++c) {
            aeq(row, base + c) = end_basis[c];
        }
        beq(row) = waypoint_values[segment_idx + 1U];
        ++row;
    }

    for (int derivative = 1; derivative <= 2; ++derivative) {
        const auto basis = polynomial_basis(0.0, derivative);
        for (int c = 0; c < kCoefficientCount; ++c) {
            aeq(row, c) = basis[c];
        }
        ++row;
    }

    const size_t last_segment = segment_count - 1U;
    const int last_base = static_cast<int>(last_segment * kCoefficientCount);
    const double last_duration = this->segment_times_s_[last_segment];
    for (int derivative = 1; derivative <= 2; ++derivative) {
        const auto basis = polynomial_basis(last_duration, derivative);
        for (int c = 0; c < kCoefficientCount; ++c) {
            aeq(row, last_base + c) = basis[c];
        }
        ++row;
    }

    for (size_t segment_idx = 0; segment_idx + 1U < segment_count; ++segment_idx) {
        const int left_base = static_cast<int>(segment_idx * kCoefficientCount);
        const int right_base = static_cast<int>((segment_idx + 1U) * kCoefficientCount);
        const double left_duration = this->segment_times_s_[segment_idx];
        for (int derivative = 1; derivative <= 3; ++derivative) {
            const auto left_basis = polynomial_basis(left_duration, derivative);
            const auto right_basis = polynomial_basis(0.0, derivative);
            for (int c = 0; c < kCoefficientCount; ++c) {
                aeq(row, left_base + c) = left_basis[c];
                aeq(row, right_base + c) = -right_basis[c];
            }
            ++row;
        }
    }

    if (row != constraint_count) {
        return false;
    }

    Eigen::MatrixXd kkt =
        Eigen::MatrixXd::Zero(variable_count + constraint_count, variable_count + constraint_count);
    kkt.topLeftCorner(variable_count, variable_count) = q;
    kkt.topRightCorner(variable_count, constraint_count) = aeq.transpose();
    kkt.bottomLeftCorner(constraint_count, variable_count) = aeq;

    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(variable_count + constraint_count);
    rhs.tail(constraint_count) = beq;

    Eigen::LDLT<Eigen::MatrixXd> ldlt;
    ldlt.compute(kkt);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }

    const Eigen::VectorXd solution = ldlt.solve(rhs);
    if (ldlt.info() != Eigen::Success || !solution.allFinite()) {
        return false;
    }

    const Eigen::VectorXd coefficients = solution.head(variable_count);
    segment_coeffs.resize(segment_count);
    for (size_t segment_idx = 0; segment_idx < segment_count; ++segment_idx) {
        const int base = static_cast<int>(segment_idx * kCoefficientCount);
        for (int c = 0; c < kCoefficientCount; ++c) {
            segment_coeffs[segment_idx][c] = coefficients(base + c);
        }
    }
    return true;
}

size_t TrajectoryPlanner::time_segment_index(double time_s) const {
    if (this->segment_times_s_.empty()) {
        return 0U;
    }

    const double clamped_time = std::clamp(time_s, 0.0, this->total_time_s_);
    if (clamped_time >= this->total_time_s_) {
        return this->segment_times_s_.size() - 1U;
    }

    const auto it = std::upper_bound(this->cumulative_times_s_.begin(),
                                     this->cumulative_times_s_.end(), clamped_time);
    if (it == this->cumulative_times_s_.begin()) {
        return 0U;
    }
    const size_t idx =
        static_cast<size_t>(std::distance(this->cumulative_times_s_.begin(), it) - 1);
    return std::min(idx, this->segment_times_s_.size() - 1U);
}

double
TrajectoryPlanner::evaluate_axis(const std::vector<std::array<double, kCoefficientCount>> &coeffs,
                                 size_t segment_idx, double local_time_s,
                                 int derivative_order) const {
    if (segment_idx >= coeffs.size()) {
        return 0.0;
    }
    const auto basis = polynomial_basis(local_time_s, derivative_order);
    double value = 0.0;
    for (int c = 0; c < kCoefficientCount; ++c) {
        value += coeffs[segment_idx][c] * basis[c];
    }
    return value;
}

void TrajectoryPlanner::evaluate_state(double time_s, std::array<double, 3> &position,
                                       std::array<double, 3> &velocity) const {
    position = {0.0, 0.0, 0.0};
    velocity = {0.0, 0.0, 0.0};
    if (this->path_points_map_.empty()) {
        return;
    }
    if (this->path_points_map_.size() == 1U) {
        position[0] = this->path_points_map_.front().pose.position.x;
        position[1] = this->path_points_map_.front().pose.position.y;
        position[2] = this->path_points_map_.front().pose.position.z;
        return;
    }
    if (this->segment_times_s_.empty()) {
        return;
    }

    const double clamped_time = std::clamp(time_s, 0.0, this->total_time_s_);
    const size_t segment_idx = this->time_segment_index(clamped_time);
    const double segment_start = this->cumulative_times_s_[segment_idx];
    const double local_time =
        std::clamp(clamped_time - segment_start, 0.0, this->segment_times_s_[segment_idx]);

    position[0] = this->evaluate_axis(this->coeffs_x_, segment_idx, local_time, 0);
    position[1] = this->evaluate_axis(this->coeffs_y_, segment_idx, local_time, 0);
    position[2] = this->evaluate_axis(this->coeffs_z_, segment_idx, local_time, 0);
    velocity[0] = this->evaluate_axis(this->coeffs_x_, segment_idx, local_time, 1);
    velocity[1] = this->evaluate_axis(this->coeffs_y_, segment_idx, local_time, 1);
    velocity[2] = this->evaluate_axis(this->coeffs_z_, segment_idx, local_time, 1);
}

void TrajectoryPlanner::build_samples() {
    this->samples_.clear();
    this->total_length_m_ = 0.0;
    if (this->path_points_map_.empty()) {
        return;
    }

    auto append_sample = [this](double time_s, const std::array<double, 3> &position,
                                const std::array<double, 3> &velocity) {
        SamplePoint sample;
        sample.time_s = time_s;
        sample.position = position;
        sample.velocity = velocity;
        if (!this->samples_.empty()) {
            const auto &prev = this->samples_.back();
            const double dx = sample.position[0] - prev.position[0];
            const double dy = sample.position[1] - prev.position[1];
            const double dz = sample.position[2] - prev.position[2];
            sample.arc_length_m = prev.arc_length_m + std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        this->samples_.push_back(sample);
    };

    if (this->path_points_map_.size() == 1U) {
        std::array<double, 3> position{
            this->path_points_map_.front().pose.position.x,
            this->path_points_map_.front().pose.position.y,
            this->path_points_map_.front().pose.position.z,
        };
        std::array<double, 3> velocity{0.0, 0.0, 0.0};
        append_sample(0.0, position, velocity);
        return;
    }

    constexpr double kSampleStepTimeS = 0.4;
    constexpr size_t kMinSamplesPerSegment = 1;
    for (size_t segment_idx = 0; segment_idx < this->segment_times_s_.size(); ++segment_idx) {
        const double duration_s = this->segment_times_s_[segment_idx];
        const size_t sample_count =
            std::max(kMinSamplesPerSegment,
                     static_cast<size_t>(std::ceil(duration_s / kSampleStepTimeS)) + 1U);
        for (size_t j = 0; j < sample_count; ++j) {
            if (segment_idx > 0U && j == 0U) {
                continue;
            }
            const double ratio = static_cast<double>(j) / static_cast<double>(sample_count - 1U);
            const double local_time = duration_s * ratio;
            const double global_time = this->cumulative_times_s_[segment_idx] + local_time;

            std::array<double, 3> position{};
            std::array<double, 3> velocity{};
            this->evaluate_state(global_time, position, velocity);
            append_sample(global_time, position, velocity);
        }
    }
    if (this->samples_.empty()) {
        return;
    }
    this->total_length_m_ = this->samples_.back().arc_length_m;
}

double TrajectoryPlanner::project_time_from_xy(double x, double y, double *arc_length_m) const {
    if (this->samples_.empty()) {
        if (arc_length_m != nullptr) {
            *arc_length_m = 0.0;
        }
        return 0.0;
    }
    if (this->samples_.size() == 1U) {
        if (arc_length_m != nullptr) {
            *arc_length_m = 0.0;
        }
        return this->samples_.front().time_s;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    double best_time_s = this->samples_.front().time_s;
    double best_arc_length = this->samples_.front().arc_length_m;

    for (size_t i = 0; i + 1U < this->samples_.size(); ++i) {
        const auto &sample0 = this->samples_[i];
        const auto &sample1 = this->samples_[i + 1U];
        const double seg_dx = sample1.position[0] - sample0.position[0];
        const double seg_dy = sample1.position[1] - sample0.position[1];
        const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
        if (seg_len_sq <= kEpsilon) {
            continue;
        }

        const double rel_x = x - sample0.position[0];
        const double rel_y = y - sample0.position[1];
        const double alpha = std::clamp((rel_x * seg_dx + rel_y * seg_dy) / seg_len_sq, 0.0, 1.0);
        const double proj_x = sample0.position[0] + alpha * seg_dx;
        const double proj_y = sample0.position[1] + alpha * seg_dy;
        const double distance_sq = (x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y);
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_time_s = sample0.time_s + alpha * (sample1.time_s - sample0.time_s);
            best_arc_length =
                sample0.arc_length_m + alpha * (sample1.arc_length_m - sample0.arc_length_m);
        }
    }

    if (arc_length_m != nullptr) {
        *arc_length_m = best_arc_length;
    }
    return best_time_s;
}

double TrajectoryPlanner::time_at_arc(double arc_length_m) const {
    if (this->samples_.empty()) {
        return 0.0;
    }

    const double clamped_arc = std::clamp(arc_length_m, 0.0, this->total_length_m_);
    const auto it = std::lower_bound(
        this->samples_.begin(), this->samples_.end(), clamped_arc,
        [](const SamplePoint &sample, double arc) { return sample.arc_length_m < arc; });
    if (it == this->samples_.begin()) {
        return this->samples_.front().time_s;
    }
    if (it == this->samples_.end()) {
        return this->samples_.back().time_s;
    }

    const auto &upper = *it;
    const auto &lower = *(it - 1);
    const double ds = std::max(upper.arc_length_m - lower.arc_length_m, kEpsilon);
    const double alpha = (clamped_arc - lower.arc_length_m) / ds;
    return lower.time_s + alpha * (upper.time_s - lower.time_s);
}

double TrajectoryPlanner::speed_at_time(double time_s) const {
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    this->evaluate_state(time_s, position, velocity);
    return std::hypot(velocity[0], velocity[1]);
}

bool TrajectoryPlanner::heading_direction_at_time(double time_s, double &dir_x,
                                                  double &dir_y) const {
    dir_x = 1.0;
    dir_y = 0.0;

    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    this->evaluate_state(time_s, position, velocity);
    const double speed = std::hypot(velocity[0], velocity[1]);
    if (speed > kEpsilon) {
        dir_x = velocity[0] / speed;
        dir_y = velocity[1] / speed;
        return true;
    }

    if (this->samples_.size() < 2U) {
        return false;
    }

    const auto it =
        std::lower_bound(this->samples_.begin(), this->samples_.end(), time_s,
                         [](const SamplePoint &sample, double t) { return sample.time_s < t; });
    size_t idx = 0U;
    if (it == this->samples_.end()) {
        idx = this->samples_.size() - 1U;
    } else {
        idx = static_cast<size_t>(std::distance(this->samples_.begin(), it));
    }

    auto extract_dir = [this, &dir_x, &dir_y](size_t from, size_t to) {
        const double dx = this->samples_[to].position[0] - this->samples_[from].position[0];
        const double dy = this->samples_[to].position[1] - this->samples_[from].position[1];
        const double norm = std::hypot(dx, dy);
        if (norm <= kEpsilon) {
            return false;
        }
        dir_x = dx / norm;
        dir_y = dy / norm;
        return true;
    };

    if (idx > 0U && extract_dir(idx - 1U, idx)) {
        return true;
    }
    if (idx + 1U < this->samples_.size() && extract_dir(idx, idx + 1U)) {
        return true;
    }
    if (idx > 1U && extract_dir(idx - 2U, idx - 1U)) {
        return true;
    }
    if (idx + 2U < this->samples_.size() && extract_dir(idx + 1U, idx + 2U)) {
        return true;
    }
    return false;
}

bool TrajectoryPlanner::sample_waypoint(const geometry_msgs::msg::PoseStamped &current_pose_map,
                                        geometry_msgs::msg::PoseStamped &waypoint_map,
                                        geometry_msgs::msg::Twist &waypoint_twist_map) const {
    waypoint_map = geometry_msgs::msg::PoseStamped{};
    waypoint_twist_map = geometry_msgs::msg::Twist{};
    if (!this->is_ready()) {
        return false;
    }
    if (!current_pose_map.header.frame_id.empty() &&
        current_pose_map.header.frame_id != this->frame_id_) {
        return false;
    }

    if (this->path_points_map_.size() == 1U) {
        waypoint_map = this->path_points_map_.front();
        waypoint_map.header.frame_id = this->frame_id_;
        waypoint_map.header.stamp = current_pose_map.header.stamp;
        waypoint_map.pose.orientation = yaw_to_quat(0.0);
        return true;
    }

    double current_arc_length = 0.0;
    const double current_time = this->project_time_from_xy(
        current_pose_map.pose.position.x, current_pose_map.pose.position.y, &current_arc_length);
    const double speed_now = this->speed_at_time(current_time);
    const double lookahead_m =
        std::max(this->limits_.min_lookahead_m, this->limits_.lookahead_time_s * speed_now);
    const double target_arc = std::min(this->total_length_m_, current_arc_length + lookahead_m);
    const double target_time = this->time_at_arc(target_arc);

    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    this->evaluate_state(target_time, position, velocity);

    const double horizontal_speed = std::hypot(velocity[0], velocity[1]);
    if (horizontal_speed > this->limits_.max_speed_mps && horizontal_speed > kEpsilon) {
        const double scale = this->limits_.max_speed_mps / horizontal_speed;
        velocity[0] *= scale;
        velocity[1] *= scale;
        velocity[2] *= scale;
    }

    double dir_x = 1.0;
    double dir_y = 0.0;
    this->heading_direction_at_time(target_time, dir_x, dir_y);

    waypoint_map.header.frame_id = this->frame_id_;
    waypoint_map.header.stamp = current_pose_map.header.stamp;
    waypoint_map.pose.position.x = position[0];
    waypoint_map.pose.position.y = position[1];
    waypoint_map.pose.position.z = position[2];
    waypoint_map.pose.orientation = yaw_to_quat(std::atan2(dir_y, dir_x));

    waypoint_twist_map.linear.x = velocity[0];
    waypoint_twist_map.linear.y = velocity[1];
    waypoint_twist_map.linear.z = velocity[2];
    return true;
}

} // namespace red::navigator
