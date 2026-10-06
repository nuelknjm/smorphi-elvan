#pragma once

#include "Types.hpp"
#include <vector>

namespace smorphi {

/// Minimal IMU reading. A real driver would also expose raw accelerometer /
/// gyroscope axes and a covariance estimate; this is trimmed to what the
/// localization layer in this project consumes.
struct IMUReading {
    double yaw_rate{0.0};        // rad/s, gyroscope z-axis
    double heading{0.0};         // rad, IMU's own internally-fused absolute heading
    double linear_accel_x{0.0};  // m/s^2, robot frame
    double linear_accel_y{0.0};  // m/s^2, robot frame
};

/// A single 2D LiDAR scan. `ranges` is the raw polar scan (index -> angle);
/// `obstacle_points` are already-clustered obstacle points in the robot
/// frame, as a pre-processed convenience for the costmap layer.
struct LidarScan {
    std::vector<double> ranges;
    std::vector<Point2D> obstacle_points;
};

/// Hardware abstraction layer. Localization, Planning and the FSM only
/// ever depend on these two interfaces -- never on a concrete driver --
/// so swapping in the real Smorphi IMU/LiDAR drivers later requires no
/// change anywhere else in the navigation stack (Dependency Inversion).
class IIMUSensor {
public:
    virtual ~IIMUSensor() = default;
    virtual IMUReading read() = 0;
};

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;
    virtual LidarScan read() = 0;
};

/// Placeholder drivers used until the real Smorphi hardware interfaces are
/// wired in by the Hardware Division. Swap these for implementations that
/// talk to the actual IMU/LiDAR (I2C/SPI, serial, ROS2 topic, etc.).
class SimulatedIMU : public IIMUSensor {
public:
    IMUReading read() override;
};

class SimulatedLidar : public ILidarSensor {
public:
    LidarScan read() override;
};

} // namespace smorphi
