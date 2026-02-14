#pragma once

#include <array>

#include <Eigen/Dense>

namespace red::geometric_controller {

std::array<double, 4> wrench_to_motor_speeds(double force, const Eigen::Vector3d &torque,
                                             const Eigen::Matrix<double, 4, 3> &rotor_positions,
                                             double rotor_cf, double rotor_cd,
                                             const Eigen::Vector4d &yaw_signs,
                                             double motor_max_rot_velocity);

} // namespace red::geometric_controller
