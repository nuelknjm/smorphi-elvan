#include "Localization.hpp"
#include <cmath>
#include <algorithm>
#include <random>

namespace smorphi {

namespace {
std::mt19937& rng() {
    static std::mt19937 generator{std::random_device{}()};
    return generator;
}
double gaussianNoise(double stddev) {
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(rng());
}
} // namespace

// ---------------------------------------------------------------------
// ExtendedKalmanFilter
// ---------------------------------------------------------------------

ExtendedKalmanFilter::ExtendedKalmanFilter(const Pose2D& initial_pose)
    : state_(initial_pose) {
    // Small, confident prior; a production system would tune this from
    // sensor datasheets / bench calibration rather than hard-coding it.
    covariance_ = {0.01, 0, 0,
                   0, 0.01, 0,
                   0, 0, 0.01};
}

void ExtendedKalmanFilter::predict(const IMUReading& imu, const HolonomicVelocity& last_cmd, double dt) {
    // Holonomic motion model (see header for the full equations). We
    // trust the IMU's yaw_rate over the commanded omega where available,
    // since it is a direct physical measurement rather than an open-loop
    // command.
    const double theta = state_.theta;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    state_.x += (last_cmd.vx * c - last_cmd.vy * s) * dt;
    state_.y += (last_cmd.vx * s + last_cmd.vy * c) * dt;
    state_.theta += imu.yaw_rate * dt;

    // Simplified covariance growth (diagonal only). A production
    // implementation would compute the full Jacobian F of the motion
    // model and propagate P' = F*P*F^T + Q with a linear-algebra library
    // (e.g. Eigen) rather than this scalar stand-in.
    constexpr double process_noise = 0.005;
    covariance_[0] += process_noise * dt;
    covariance_[4] += process_noise * dt;
    covariance_[8] += process_noise * dt * 0.5;
}

void ExtendedKalmanFilter::correct(const Pose2D& measurement, const std::array<double, 9>& measurement_covariance) {
    // Simplified per-axis scalar Kalman gain (K_i = P_i / (P_i + R_i)).
    // A production implementation would use the full 3x3 matrix form
    // documented in the header (K = P*H^T*(H*P*H^T + R)^-1).
    auto blend = [](double prior, double prior_var, double meas, double meas_var) {
        const double k = prior_var / (prior_var + meas_var);
        return prior + k * (meas - prior);
    };

    state_.x = blend(state_.x, covariance_[0], measurement.x, measurement_covariance[0]);
    state_.y = blend(state_.y, covariance_[4], measurement.y, measurement_covariance[4]);
    state_.theta = blend(state_.theta, covariance_[8], measurement.theta, measurement_covariance[8]);

    covariance_[0] *= 0.5;
    covariance_[4] *= 0.5;
    covariance_[8] *= 0.5;
}

// ---------------------------------------------------------------------
// ParticleFilter
// ---------------------------------------------------------------------

ParticleFilter::ParticleFilter(const Pose2D& initial_pose, std::size_t num_particles) {
    particles_.resize(num_particles);
    for (auto& p : particles_) {
        p.pose = initial_pose;
        p.weight = 1.0 / static_cast<double>(num_particles);
    }
}

void ParticleFilter::predict(const HolonomicVelocity& last_cmd, double dt) {
    constexpr double pos_noise_stddev = 0.02;
    constexpr double theta_noise_stddev = 0.01;

    for (auto& p : particles_) {
        const double c = std::cos(p.pose.theta);
        const double s = std::sin(p.pose.theta);
        p.pose.x += (last_cmd.vx * c - last_cmd.vy * s) * dt + gaussianNoise(pos_noise_stddev);
        p.pose.y += (last_cmd.vx * s + last_cmd.vy * c) * dt + gaussianNoise(pos_noise_stddev);
        p.pose.theta += last_cmd.omega * dt + gaussianNoise(theta_noise_stddev);
    }
}

double ParticleFilter::scanLikelihood(const Pose2D& hypothesis, const LidarScan& scan) const {
    // TODO: a real implementation ray-casts from `hypothesis` against the
    // known map and compares expected vs. actual ranges (a beam model or
    // likelihood-field model). Placeholder: closer observed obstacle
    // points imply a higher-likelihood hypothesis -- a stand-in scoring
    // signal with the right shape (always positive, higher is better)
    // for resampling to work against.
    double score = 1.0;
    for (const auto& obstacle : scan.obstacle_points) {
        const double dx = obstacle.x - hypothesis.x;
        const double dy = obstacle.y - hypothesis.y;
        score += 1.0 / (1.0 + std::sqrt(dx * dx + dy * dy));
    }
    return score;
}

void ParticleFilter::updateWithScan(const LidarScan& scan) {
    double weight_sum = 0.0;
    for (auto& p : particles_) {
        p.weight = scanLikelihood(p.pose, scan);
        weight_sum += p.weight;
    }
    if (weight_sum > 0.0) {
        for (auto& p : particles_) {
            p.weight /= weight_sum;
        }
    }
    resample();
}

void ParticleFilter::resample() {
    // Systematic resampling: O(N), lower variance than naive
    // roulette-wheel sampling.
    if (particles_.empty()) return;

    std::vector<Particle> resampled;
    resampled.reserve(particles_.size());

    const double step = 1.0 / static_cast<double>(particles_.size());
    std::uniform_real_distribution<double> start_dist(0.0, step);
    const double cursor = start_dist(rng());
    double cumulative = particles_.front().weight;
    std::size_t i = 0;

    for (std::size_t j = 0; j < particles_.size(); ++j) {
        const double target = cursor + static_cast<double>(j) * step;
        while (cumulative < target && i < particles_.size() - 1) {
            ++i;
            cumulative += particles_[i].weight;
        }
        resampled.push_back(particles_[i]);
        resampled.back().weight = step;
    }
    particles_ = std::move(resampled);
}

Pose2D ParticleFilter::estimatePose() const {
    double x = 0, y = 0, sin_sum = 0, cos_sum = 0;
    for (const auto& p : particles_) {
        x += p.pose.x * p.weight;
        y += p.pose.y * p.weight;
        // Heading is circular; average via sin/cos sums rather than
        // averaging the angle value directly.
        sin_sum += std::sin(p.pose.theta) * p.weight;
        cos_sum += std::cos(p.pose.theta) * p.weight;
    }
    return Pose2D{x, y, std::atan2(sin_sum, cos_sum)};
}

// ---------------------------------------------------------------------
// LocalizationEngine
// ---------------------------------------------------------------------

LocalizationEngine::LocalizationEngine(const Pose2D& initial_pose)
    : ekf_(initial_pose), particle_filter_(initial_pose) {}

void LocalizationEngine::update(const IMUReading& imu, const LidarScan& scan,
                                 const HolonomicVelocity& last_cmd, double dt) {
    // Fast path: EKF predicts every cycle from IMU + commanded velocity.
    ekf_.predict(imu, last_cmd, dt);

    // Slow path: the Particle Filter predicts + corrects from LiDAR only
    // periodically, since scan matching across hundreds of particles is
    // the expensive step; its result then corrects the EKF.
    particle_filter_.predict(last_cmd, dt);
    ++cycles_since_pf_correction_;
    if (cycles_since_pf_correction_ >= kPfCorrectionPeriod) {
        particle_filter_.updateWithScan(scan);
        const Pose2D pf_estimate = particle_filter_.estimatePose();
        const std::array<double, 9> pf_covariance = {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02};
        ekf_.correct(pf_estimate, pf_covariance);
        cycles_since_pf_correction_ = 0;
    }
}

} // namespace smorphi
