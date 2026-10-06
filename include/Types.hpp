#pragma once

namespace smorphi {

/// 2D pose (position + heading) expressed in the global/world frame.
struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0}; // heading, radians, CCW positive

    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_ = 0.0) : x(x_), y(y_), theta(theta_) {}
};

/// Holonomic velocity command. Unlike a differential-drive robot (v, omega),
/// a mecanum/omni-wheel platform like Smorphi can command Vx and Vy
/// independently of its heading, which is what makes it "holonomic".
struct HolonomicVelocity {
    double vx{0.0};    // m/s, robot frame
    double vy{0.0};    // m/s, robot frame
    double omega{0.0}; // rad/s
};

/// Simple 2D point, used for grid cells, waypoints and targets.
struct Point2D {
    double x{0.0};
    double y{0.0};
};

} // namespace smorphi
