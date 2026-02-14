#include "red_geometric_controller/transform.hpp"

#include <cmath>

namespace red::geometric_controller {

double roll_pitch_to_tilt(double roll, double pitch) {
    double cos_val = std::cos(pitch) * std::cos(roll);
    if (cos_val > 1.0) {
        cos_val = 1.0;
    } else if (cos_val < -1.0) {
        cos_val = -1.0;
    }
    return std::acos(cos_val);
}

Eigen::Matrix3d quat_to_rotmat(double w, double x, double y, double z) {
    const double n = w * w + x * x + y * y + z * z;
    const double s = 2.0 / n;
    const double wx = s * w * x;
    const double wy = s * w * y;
    const double wz = s * w * z;
    const double xx = s * x * x;
    const double xy = s * x * y;
    const double xz = s * x * z;
    const double yy = s * y * y;
    const double yz = s * y * z;
    const double zz = s * z * z;

    Eigen::Matrix3d rot;
    rot << 1.0 - (yy + zz), xy - wz, xz + wy, xy + wz, 1.0 - (xx + zz), yz - wx, xz - wy, yz + wx,
        1.0 - (xx + yy);
    return rot;
}

std::tuple<double, double, double> quat_to_euler(double w, double x, double y, double z) {
    constexpr double kPi = 3.14159265358979323846;
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(kPi / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

std::tuple<double, double, double, double> euler_to_quat(double roll, double pitch, double yaw) {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    const double w = cr * cp * cy + sr * sp * sy;
    const double x = sr * cp * cy - cr * sp * sy;
    const double y = cr * sp * cy + sr * cp * sy;
    const double z = cr * cp * sy - sr * sp * cy;

    return {w, x, y, z};
}

} // namespace red::geometric_controller
