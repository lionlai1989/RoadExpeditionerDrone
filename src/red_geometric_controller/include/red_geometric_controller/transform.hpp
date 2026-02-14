#pragma once

#include <tuple>

#include <Eigen/Dense>

namespace red::geometric_controller {

double roll_pitch_to_tilt(double roll, double pitch);

Eigen::Matrix3d quat_to_rotmat(double w, double x, double y, double z);

std::tuple<double, double, double> quat_to_euler(double w, double x, double y, double z);

std::tuple<double, double, double, double> euler_to_quat(double roll, double pitch, double yaw);

} // namespace red::geometric_controller
