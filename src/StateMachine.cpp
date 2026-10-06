#include "StateMachine.hpp"
#include <iostream>
#include <thread>
#include <cmath>

namespace smorphi {

namespace {
constexpr double kControlPeriodS = 0.1; // 10 Hz control loop
}

SmorphiNavigationFSM::SmorphiNavigationFSM(std::unique_ptr<IIMUSensor> imu,
                                            std::unique_ptr<ILidarSensor> lidar,
                                            std::deque<Point2D> target_queue,
                                            Costmap2D costmap)
    : imu_(std::move(imu)), lidar_(std::move(lidar)),
      costmap_(std::move(costmap)), target_queue_(std::move(target_queue)) {}

void SmorphiNavigationFSM::initialize() {
    std::cout << "[INITIALIZE] Starting pose set to (0, 0). Loaded "
              << target_queue_.size() << " target(s) into the queue.\n";

    localization_ = std::make_unique<LocalizationEngine>(Pose2D{0.0, 0.0, 0.0});
    global_planner_ = std::make_unique<AStarPlanner>(costmap_);
    local_planner_ = std::make_unique<HolonomicDWA>();

    state_ = RobotState::SENSE;
}

void SmorphiNavigationFSM::sense() {
    last_imu_reading_ = imu_->read();
    last_lidar_scan_ = lidar_->read();
    costmap_.updateFromScan(localization_->currentPose(), last_lidar_scan_);

    state_ = RobotState::THINK;
}

void SmorphiNavigationFSM::think() {
    localization_->update(last_imu_reading_, last_lidar_scan_, last_cmd_velocity_, kControlPeriodS);

    if (target_queue_.empty()) {
        state_ = RobotState::SHUTDOWN;
        return;
    }

    if (dwelling_) {
        // Already stopped at the target -- just waiting out the dwell
        // timer; nothing to plan this cycle.
        state_ = RobotState::ACT;
        return;
    }

    if (isAtCurrentTarget()) {
        beginDwell();
        state_ = RobotState::ACT;
        return;
    }

    // Replan globally every cycle (acceptable at this arena's scale; a
    // larger map would replan on a timer or only when the path is
    // invalidated by newly-sensed obstacles).
    const Point2D& target = target_queue_.front();
    const Pose2D& current_pose = localization_->currentPose();
    current_global_path_ = global_planner_->plan({current_pose.x, current_pose.y}, target);
    path_index_ = 0;

    if (current_global_path_.empty()) {
        std::cout << "[THINK] A* found no path to target -- holding position.\n";
        last_cmd_velocity_ = HolonomicVelocity{};
        state_ = RobotState::ACT;
        return;
    }

    // The local planner tracks the next waypoint along the global path
    // (not the final target directly), which is what lets DWA react to
    // obstacles the global plan didn't know about yet.
    const Point2D& next_waypoint = current_global_path_[path_index_];
    last_cmd_velocity_ = local_planner_->computeVelocity(
        current_pose, last_cmd_velocity_, next_waypoint, costmap_);

    state_ = RobotState::ACT;
}

void SmorphiNavigationFSM::act() {
    if (dwelling_) {
        last_cmd_velocity_ = HolonomicVelocity{}; // full stop
        std::cout << "[ACT] Dwelling at target (" << target_queue_.front().x << ", "
                  << target_queue_.front().y << ")...\n";

        if (dwellComplete()) {
            std::cout << "[ACT] 5s dwell complete -- advancing to next target.\n";
            target_queue_.pop_front();
            dwelling_ = false;
        }
    } else {
        const Pose2D& pose = localization_->currentPose();
        std::cout << "[ACT] Pose (" << pose.x << ", " << pose.y << ", " << pose.theta
                  << ") -> cmd (vx=" << last_cmd_velocity_.vx << ", vy=" << last_cmd_velocity_.vy
                  << ", w=" << last_cmd_velocity_.omega << ")\n";
        // TODO: send last_cmd_velocity_ to the Hardware Division's
        // mecanum wheel driver / motor controller here.
    }

    state_ = target_queue_.empty() ? RobotState::SHUTDOWN : RobotState::SENSE;
}

bool SmorphiNavigationFSM::isAtCurrentTarget(double tolerance_m) const {
    if (target_queue_.empty()) return false;
    const Pose2D& pose = localization_->currentPose();
    const Point2D& target = target_queue_.front();
    return std::hypot(target.x - pose.x, target.y - pose.y) <= tolerance_m;
}

void SmorphiNavigationFSM::beginDwell() {
    dwelling_ = true;
    dwell_start_ = std::chrono::steady_clock::now();
}

bool SmorphiNavigationFSM::dwellComplete() const {
    return std::chrono::steady_clock::now() - dwell_start_ >= kDwellDuration;
}

void SmorphiNavigationFSM::run() {
    initialize();

    while (state_ != RobotState::SHUTDOWN) {
        switch (state_) {
            case RobotState::SENSE: sense(); break;
            case RobotState::THINK: think(); break;
            case RobotState::ACT: act(); break;
            default: break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(kControlPeriodS));
    }

    std::cout << "[SHUTDOWN] All targets reached. Navigation complete.\n";
}

} // namespace smorphi
