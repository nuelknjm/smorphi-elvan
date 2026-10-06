#pragma once

#include "Types.hpp"
#include "Sensors.hpp"
#include "Localization.hpp"
#include "Planning.hpp"
#include <deque>
#include <vector>
#include <memory>
#include <chrono>

namespace smorphi {

enum class RobotState {
    INITIALIZE,
    SENSE,
    THINK,
    ACT,
    SHUTDOWN
};

/// Top-level orchestrator. Owns the sensors, the localization engine, the
/// global/local planners and the target queue, and drives the
/// Sense-Think-Act loop. Each private method below corresponds 1:1 to one
/// state in the FSM spec, so run()'s control flow mirrors the state
/// diagram directly -- this is the piece worth reading side-by-side with
/// the FSM diagram during the presentation.
class SmorphiNavigationFSM {
public:
    SmorphiNavigationFSM(std::unique_ptr<IIMUSensor> imu,
                          std::unique_ptr<ILidarSensor> lidar,
                          std::deque<Point2D> target_queue,
                          Costmap2D costmap);

    /// Runs INITIALIZE once, then loops SENSE -> THINK -> ACT until the
    /// target queue is empty, at which point it transitions to SHUTDOWN.
    void run();

private:
    void initialize();
    void sense();
    void think();
    void act();

    bool isAtCurrentTarget(double tolerance_m = 0.05) const;
    void beginDwell();
    bool dwellComplete() const;

    RobotState state_{RobotState::INITIALIZE};

    std::unique_ptr<IIMUSensor> imu_;
    std::unique_ptr<ILidarSensor> lidar_;

    std::unique_ptr<LocalizationEngine> localization_;
    std::unique_ptr<AStarPlanner> global_planner_;
    std::unique_ptr<HolonomicDWA> local_planner_;
    Costmap2D costmap_;

    std::deque<Point2D> target_queue_;
    std::vector<Point2D> current_global_path_;
    std::size_t path_index_{0};

    IMUReading last_imu_reading_;
    LidarScan last_lidar_scan_;
    HolonomicVelocity last_cmd_velocity_;

    bool dwelling_{false};
    std::chrono::steady_clock::time_point dwell_start_;
    static constexpr std::chrono::seconds kDwellDuration{5};
};

} // namespace smorphi
