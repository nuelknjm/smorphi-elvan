#pragma once

#include "Types.hpp"
#include "Sensors.hpp"
#include <array>
#include <vector>
#include <cstddef>

namespace smorphi {

/// Extended Kalman Filter over state [x, y, theta].
///
/// Role in the fusion architecture: the FAST, continuous estimator.
/// predict() runs every control cycle using the IMU + the last commanded
/// holonomic velocity as the motion model; correct() runs whenever a
/// lower-rate, globally-consistent pose estimate becomes available (here,
/// from the Particle Filter) and pulls the EKF state back toward it,
/// bounding the drift that pure IMU integration would otherwise accumulate.
class ExtendedKalmanFilter {
public:
    explicit ExtendedKalmanFilter(const Pose2D& initial_pose);

    /// Predict step.
    /// State transition model (holonomic):
    ///   x'     = x + (vx*cos(theta) - vy*sin(theta)) * dt
    ///   y'     = y + (vx*sin(theta) + vy*cos(theta)) * dt
    ///   theta' = theta + omega * dt
    /// Covariance update: P' = F * P * F^T + Q, where F is the Jacobian of
    /// the above w.r.t. [x, y, theta] and Q is process noise.
    void predict(const IMUReading& imu, const HolonomicVelocity& last_cmd, double dt);

    /// Correct step (measurement update).
    /// Innovation:    y = measurement - state_
    /// Kalman gain:   K = P * H^T * (H * P * H^T + R)^-1
    /// State update:  state_ = state_ + K * y
    /// Covariance:    P = (I - K * H) * P
    /// H = identity here because the Particle Filter already outputs a
    /// pose in the same state space.
    void correct(const Pose2D& measurement, const std::array<double, 9>& measurement_covariance);

    const Pose2D& state() const { return state_; }
    const std::array<double, 9>& covariance() const { return covariance_; }

private:
    Pose2D state_;
    std::array<double, 9> covariance_{}; // row-major 3x3, over [x, y, theta]
};

/// Particle Filter (Monte Carlo Localization).
///
/// Role in the fusion architecture: the SLOW, globally-robust estimator.
/// Maintains a population of pose hypotheses ("particles"); each
/// prediction step propagates every particle through the same motion
/// model as the EKF with added process noise, and each measurement update
/// re-weights particles by how well their *expected* LiDAR scan (from the
/// known map) matches the *actual* scan, then resamples. Unlike the EKF,
/// a PF is not confined to a single Gaussian belief, so it can recover
/// from large localization errors (e.g. the "kidnapped robot" problem)
/// that the EKF alone cannot.
class ParticleFilter {
public:
    explicit ParticleFilter(const Pose2D& initial_pose, std::size_t num_particles = 500);

    void predict(const HolonomicVelocity& last_cmd, double dt);
    void updateWithScan(const LidarScan& scan);

    /// Weighted mean pose across the particle set. Fed into
    /// ExtendedKalmanFilter::correct() as the measurement.
    Pose2D estimatePose() const;

private:
    struct Particle {
        Pose2D pose;
        double weight{1.0};
    };

    std::vector<Particle> particles_;

    void resample();
    double scanLikelihood(const Pose2D& hypothesis, const LidarScan& scan) const;
};

/// Sensor-fusion facade: wires the EKF and Particle Filter together into
/// the single pose estimate the rest of the navigation stack (global +
/// local planners) consumes, so THINK never has to know the fusion
/// internals.
///
/// Fusion schedule: EKF predicts every cycle from the IMU (fast, cheap).
/// The Particle Filter predicts + updates from LiDAR every
/// kPfCorrectionPeriod cycles (slower, more expensive -- scan matching
/// across hundreds of particles) and its estimate is fed back into the
/// EKF as a correction, which is the classic "slow-but-accurate corrects
/// fast-but-drifting" pattern used across mobile robotics.
class LocalizationEngine {
public:
    explicit LocalizationEngine(const Pose2D& initial_pose);

    void update(const IMUReading& imu, const LidarScan& scan,
                const HolonomicVelocity& last_cmd, double dt);

    const Pose2D& currentPose() const { return ekf_.state(); }

private:
    ExtendedKalmanFilter ekf_;
    ParticleFilter particle_filter_;
    int cycles_since_pf_correction_{0};
    static constexpr int kPfCorrectionPeriod = 5;
};

} // namespace smorphi
