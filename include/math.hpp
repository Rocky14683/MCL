#pragma once

#include <numbers>

inline constexpr double to_radians(double degrees) {
    return degrees * std::numbers::pi / 180;
}

inline constexpr double to_degrees(double radians) {
    return radians * 180 * std::numbers::inv_pi;
}

/**
 * [0, 2pi)
 */
inline constexpr double norm(double radians) {
    radians = fmod(radians, 2 * std::numbers::pi);
    if (radians < 0)
        radians += 2 * std::numbers::pi;
    return radians;
}

/**
 * [-pi, pi]
 */
inline constexpr double norm_delta(double radians) {
    return std::remainder(radians, 2 * std::numbers::pi);
}

inline Point2d to_point2d(const Pose2d &pose) {
    return Point2d{pose.x(), pose.y()};
}