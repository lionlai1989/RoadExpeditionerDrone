#pragma once

#include <Eigen/Dense>
#include <utility>

namespace red::geometric_controller {

constexpr double kGravity = 9.81;

// NOTE: Physical parameters defined here MUST match the values defined in
// "src/red_gz_plugin/models/x500_depth/model.sdf" and its associated files.

class QuadcopterParams {
  public:
    QuadcopterParams();

    double compute_max_accel() const;
    void calculate_wrench_limits();

    // Mass (kg) for x500_depth: base_link + 4 rotors + OakD-Lite.
    double mass{2.1253076923076923};
    // Diagonal inertia [Ixx, Iyy, Izz] for x500_depth about base_link.
    // Computed by summing base_link + 4 rotors + OakD-Lite with parallel-axis theorem.
    // Cross-terms are ignored because the controller assumes a diagonal inertia.
    Eigen::Vector3d inertia{0.03902101704770141, 0.040014996558448854, 0.044975198614403945};
    // Thrust coefficient cf or motorConstant (N / (rad/s)^2) from x500/model.sdf
    double rotor_cf{8.54858e-06};
    // Yaw torque coefficient k_m = motorConstant * momentConstant (N*m / (rad/s)^2) from
    // x500/model.sdf
    // https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/multicopter_motor_model/MulticopterMotorModel.cc
    double moment_constant{0.016};
    double rotor_cd{rotor_cf * moment_constant};
    // Maximum rotor angular velocity (rad/s) from x500/model.sdf
    double motor_max_rot_velocity{1200.0};
    // Rotor positions (x, y, z) in body frame from x500_base/model.sdf
    Eigen::Matrix<double, 4, 3> rotor_positions =
        (Eigen::Matrix<double, 4, 3>() << 0.174, -0.174, 0.06, -0.174, 0.174, 0.06, 0.174, 0.174,
         0.06, -0.174, -0.174, 0.06)
            .finished();
    Eigen::Vector4d yaw_signs{-1.0, -1.0, 1.0, 1.0};
    double max_tilt_angle{3.14159265358979323846 / 12.0};
    double max_accel{0.0};
    std::pair<double, double> force_z_limit{0.0, 0.0};
    std::pair<double, double> torque_x_limit{0.0, 0.0};
    std::pair<double, double> torque_y_limit{0.0, 0.0};
    std::pair<double, double> torque_z_limit{0.0, 0.0};
};

} // namespace red::geometric_controller
