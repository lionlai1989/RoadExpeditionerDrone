#include "red_geometric_controller/motor_mixing.hpp"

#include <algorithm>
#include <cassert>

namespace red::geometric_controller {

std::array<double, 4> wrench_to_motor_speeds(double force, const Eigen::Vector3d &torque,
                                             const Eigen::Matrix<double, 4, 3> &rotor_positions,
                                             double rotor_cf, double rotor_cd,
                                             const Eigen::Vector4d &yaw_signs,
                                             double motor_max_rot_velocity) {
    Eigen::Array4d thrusts = Eigen::Array4d::Constant(0.25 * force);

    const Eigen::Array4d x = rotor_positions.col(0).array();
    const Eigen::Array4d y = rotor_positions.col(1).array();
    const Eigen::Array4d sx =
        (x >= 0.0).select(Eigen::Array4d::Constant(1.0), Eigen::Array4d::Constant(-1.0));
    const Eigen::Array4d sy =
        (y >= 0.0).select(Eigen::Array4d::Constant(1.0), Eigen::Array4d::Constant(-1.0));
    const double arm_len = (x.square() + y.square()).sqrt().mean();
    assert(arm_len > 0.0);

    thrusts += (torque.x() / (4.0 * arm_len)) * sy;
    thrusts += (-torque.y() / (4.0 * arm_len)) * sx;

    const double c = rotor_cd / rotor_cf;
    assert(c > 0.0);
    thrusts += (torque.z() / (4.0 * c)) * yaw_signs.array();

    thrusts = thrusts.max(0.0);

    Eigen::Array4d speeds = (thrusts / rotor_cf).sqrt();
    speeds = speeds.min(motor_max_rot_velocity).max(0.0);

    std::array<double, 4> output{};
    for (int i = 0; i < 4; ++i) {
        output[i] = speeds(i);
    }
    return output;
}

} // namespace red::geometric_controller
