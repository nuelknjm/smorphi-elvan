#pragma once

#include "Types.hpp"
#include "Sensors.hpp"
#include <vector>
#include <cstdint>
#include <cstddef>

namespace smorphi {

/// Occupancy grid with an inflation layer.
///
/// Raw LiDAR obstacle points are rasterized into grid cells and marked
/// lethal (cost 255); inflateObstacles() then grows a decreasing-cost
/// "buffer zone" of `inflation_radius_m` around every lethal cell, so
/// A* is penalized for planning close to obstacles rather than only
/// forbidden from planning through them -- which keeps the global path
/// away from walls even when a technically-passable gap exists right
/// next to one.
class Costmap2D {
public:
    static constexpr uint8_t kFree = 0;
    static constexpr uint8_t kLethal = 255;

    Costmap2D(int width_cells, int height_cells, double resolution_m,
               double origin_x_m, double origin_y_m, double inflation_radius_m);

    void updateFromScan(const Pose2D& robot_pose, const LidarScan& scan);
    void inflateObstacles();

    uint8_t cost(int cx, int cy) const;
    bool isLethal(int cx, int cy) const;
    bool inBounds(int cx, int cy) const;
    void worldToGrid(double wx, double wy, int& cx, int& cy) const;
    void gridToWorld(int cx, int cy, double& wx, double& wy) const;

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

private:
    int width_, height_;
    double resolution_;
    double origin_x_, origin_y_;
    double inflation_radius_;
    std::vector<uint8_t> grid_;

    std::size_t index(int cx, int cy) const { return static_cast<std::size_t>(cy) * width_ + cx; }
};

/// A* global planner over the inflated costmap.
///
/// Produces a coarse, obstacle-aware waypoint path from the robot's
/// current cell to the target cell using an 8-connected grid search with
/// the Euclidean distance heuristic (admissible since diagonal step cost
/// is set to sqrt(2) * resolution). Deliberately dynamics-agnostic -- it
/// does not know or care that Smorphi is holonomic; that reasoning
/// belongs entirely to the local planner.
class AStarPlanner {
public:
    explicit AStarPlanner(const Costmap2D& costmap);

    /// Returns an ordered list of waypoints (world coordinates) from
    /// `start` to `goal`, or an empty vector if no plan was found.
    std::vector<Point2D> plan(const Point2D& start, const Point2D& goal) const;

private:
    const Costmap2D& costmap_;

    double heuristic(int ax, int ay, int bx, int by) const;
};

/// Holonomic Dynamic Window Approach.
///
/// Classic DWA searches a (v, omega) space for differential-drive robots.
/// Because Smorphi's mecanum/omni wheels decouple translation from
/// heading, the search space here is the 3D (Vx, Vy, Omega) velocity
/// space instead: every dynamically-feasible triplet within the current
/// "window" (bounded by max velocity and max acceleration over one
/// control cycle) is forward-simulated for a short horizon and scored on
/// goal progress, obstacle clearance (via the costmap) and velocity
/// magnitude. The highest-scoring command is sent to ACT.
class HolonomicDWA {
public:
    struct Limits {
        double max_vx{1.0}, max_vy{1.0}, max_omega{1.5};
        double max_accel_lin{1.0}, max_accel_ang{2.0};
        double sim_horizon_s{1.0};
        double time_step_s{0.1};
        int velocity_samples{7}; // per axis
    };

    HolonomicDWA() : HolonomicDWA(Limits{}) {}
    explicit HolonomicDWA(Limits limits);

    HolonomicVelocity computeVelocity(const Pose2D& current_pose,
                                        const HolonomicVelocity& current_velocity,
                                        const Point2D& next_waypoint,
                                        const Costmap2D& costmap) const;

private:
    Limits limits_;

    double scoreCandidate(const HolonomicVelocity& cmd, const Pose2D& current_pose,
                           const Point2D& next_waypoint, const Costmap2D& costmap) const;

    bool simulateForClearance(const HolonomicVelocity& cmd, const Pose2D& current_pose,
                               const Costmap2D& costmap, double& min_clearance_cost) const;
};

} // namespace smorphi
