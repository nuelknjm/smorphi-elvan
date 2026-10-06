#include "Sensors.hpp"

namespace smorphi {

IMUReading SimulatedIMU::read() {
    // TODO: replace with the real IMU driver read (e.g. I2C register read
    // on the Smorphi's onboard IMU). Returns a well-formed, near-zero
    // reading here so the rest of the stack always has valid input.
    IMUReading reading;
    reading.yaw_rate = 0.0;
    reading.heading = 0.0;
    reading.linear_accel_x = 0.0;
    reading.linear_accel_y = 0.0;
    return reading;
}

LidarScan SimulatedLidar::read() {
    // TODO: replace with the real 2D LiDAR driver read. Returns an empty
    // scan (no obstacles) so Costmap2D::updateFromScan has a valid, if
    // trivial, input to work with.
    LidarScan scan;
    return scan;
}

} // namespace smorphi
