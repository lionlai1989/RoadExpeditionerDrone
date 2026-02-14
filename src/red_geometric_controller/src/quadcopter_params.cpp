#include "red_geometric_controller/quadcopter_params.hpp"

#include <cassert>

namespace red::geometric_controller {

QuadcopterParams::QuadcopterParams() {
    max_accel = compute_max_accel();
    calculate_wrench_limits();

    assert(max_accel > 0.0);
    assert(rotor_cf > 0.0);
    assert(moment_constant > 0.0);
    assert(rotor_cd > 0.0);
    assert(motor_max_rot_velocity > 0.0);
}

double QuadcopterParams::compute_max_accel() const {
    return 4.0 * rotor_cf * (motor_max_rot_velocity * motor_max_rot_velocity) / mass;
}

void QuadcopterParams::calculate_wrench_limits() {
    const double max_thrust_per_motor =
        rotor_cf * (motor_max_rot_velocity * motor_max_rot_velocity);
    const double max_total_thrust = 4.0 * max_thrust_per_motor;
    force_z_limit = {0.0, max_total_thrust};

    double tx_max = 0.0;
    double tx_min = 0.0;
    for (int i = 0; i < rotor_positions.rows(); ++i) {
        const double y = rotor_positions(i, 1);
        if (y > 0.0) {
            tx_max += y * max_thrust_per_motor;
        } else if (y < 0.0) {
            tx_min += y * max_thrust_per_motor;
        }
    }
    torque_x_limit = {tx_min, tx_max};

    double ty_max = 0.0;
    double ty_min = 0.0;
    for (int i = 0; i < rotor_positions.rows(); ++i) {
        const double x = rotor_positions(i, 0);
        if (x < 0.0) {
            ty_max += (-x) * max_thrust_per_motor;
        } else if (x > 0.0) {
            ty_min += (-x) * max_thrust_per_motor;
        }
    }
    torque_y_limit = {ty_min, ty_max};

    const double max_moment_per_motor =
        rotor_cd * (motor_max_rot_velocity * motor_max_rot_velocity);
    double tz_max = 0.0;
    double tz_min = 0.0;
    for (int i = 0; i < yaw_signs.size(); ++i) {
        const double yaw_sign = yaw_signs(i);
        if (yaw_sign > 0.0) {
            tz_max += yaw_sign * max_moment_per_motor;
        } else if (yaw_sign < 0.0) {
            tz_min += yaw_sign * max_moment_per_motor;
        }
    }
    torque_z_limit = {tz_min, tz_max};
}

} // namespace red::geometric_controller
