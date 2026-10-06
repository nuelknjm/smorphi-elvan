#include "Planning.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

namespace smorphi {

// ---------------------------------------------------------------------
// Costmap2D
// ---------------------------------------------------------------------

Costmap2D::Costmap2D(int width_cells, int height_cells, double resolution_m,
                       double origin_x_m, double origin_y_m, double inflation_radius_m)
    : width_(width_cells), height_(height_cells), resolution_(resolution_m),
      origin_x_(origin_x_m), origin_y_(origin_y_m), inflation_radius_(inflation_radius_m),
      grid_(static_cast<std::size_t>(width_cells) * height_cells, kFree) {}

void Costmap2D::worldToGrid(double wx, double wy, int& cx, int& cy) const {
    cx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    cy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

void Costmap2D::gridToWorld(int cx, int cy, double& wx, double& wy) const {
    wx = origin_x_ + (cx + 0.5) * resolution_;
    wy = origin_y_ + (cy + 0.5) * resolution_;
}

bool Costmap2D::inBounds(int cx, int cy) const {
    return cx >= 0 && cy >= 0 && cx < width_ && cy < height_;
}

uint8_t Costmap2D::cost(int cx, int cy) const {
    if (!inBounds(cx, cy)) return kLethal; // treat out-of-map as lethal
    return grid_[index(cx, cy)];
}

bool Costmap2D::isLethal(int cx, int cy) const {
    return cost(cx, cy) == kLethal;
}

void Costmap2D::updateFromScan(const Pose2D& robot_pose, const LidarScan& scan) {
    const double c = std::cos(robot_pose.theta);
    const double s = std::sin(robot_pose.theta);

    for (const auto& pt : scan.obstacle_points) {
        // obstacle_points are in the robot frame; transform to world frame.
        const double wx = robot_pose.x + pt.x * c - pt.y * s;
        const double wy = robot_pose.y + pt.x * s + pt.y * c;

        int cx, cy;
        worldToGrid(wx, wy, cx, cy);
        if (inBounds(cx, cy)) {
            grid_[index(cx, cy)] = kLethal;
        }
    }
    inflateObstacles();
}

void Costmap2D::inflateObstacles() {
    const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    if (radius_cells <= 0) return;

    std::vector<uint8_t> inflated = grid_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[index(x, y)] != kLethal) continue;

            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    const int nx = x + dx, ny = y + dy;
                    if (!inBounds(nx, ny)) continue;

                    const double dist_cells = std::sqrt(static_cast<double>(dx * dx + dy * dy));
                    if (dist_cells > radius_cells) continue;

                    // Linear cost decay from kLethal at the obstacle to
                    // ~0 at the edge of the inflation radius.
                    const uint8_t decayed_cost = static_cast<uint8_t>(
                        kLethal * (1.0 - dist_cells / radius_cells));

                    auto& cell = inflated[index(nx, ny)];
                    if (cell != kLethal) {
                        cell = std::max(cell, decayed_cost);
                    }
                }
            }
        }
    }
    grid_ = std::move(inflated);
}

// ---------------------------------------------------------------------
// AStarPlanner
// ---------------------------------------------------------------------

AStarPlanner::AStarPlanner(const Costmap2D& costmap) : costmap_(costmap) {}

double AStarPlanner::heuristic(int ax, int ay, int bx, int by) const {
    const double dx = static_cast<double>(ax - bx);
    const double dy = static_cast<double>(ay - by);
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Point2D> AStarPlanner::plan(const Point2D& start, const Point2D& goal) const {
    int sx, sy, gx, gy;
    costmap_.worldToGrid(start.x, start.y, sx, sy);
    costmap_.worldToGrid(goal.x, goal.y, gx, gy);

    struct Node {
        int x, y;
        double f;
        bool operator>(const Node& other) const { return f > other.f; }
    };

    auto key = [&](int x, int y) { return y * costmap_.width() + x; };

    std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
    std::unordered_map<int, double> g_score;
    std::unordered_map<int, int> came_from;

    g_score[key(sx, sy)] = 0.0;
    open.push({sx, sy, heuristic(sx, sy, gx, gy)});

    const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool found = false;
    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (current.x == gx && current.y == gy) {
            found = true;
            break;
        }

        const double current_g = g_score[key(current.x, current.y)];

        for (int i = 0; i < 8; ++i) {
            const int nx = current.x + dxs[i];
            const int ny = current.y + dys[i];
            if (!costmap_.inBounds(nx, ny) || costmap_.isLethal(nx, ny)) continue;

            const double step_cost = (dxs[i] != 0 && dys[i] != 0) ? std::sqrt(2.0) : 1.0;
            // Costmap inflation adds a soft penalty on top of the base
            // step cost, biasing the path away from obstacle edges.
            const double inflation_penalty = costmap_.cost(nx, ny) / 255.0;
            const double tentative_g = current_g + step_cost + inflation_penalty * 2.0;

            const int nkey = key(nx, ny);
            if (!g_score.count(nkey) || tentative_g < g_score[nkey]) {
                g_score[nkey] = tentative_g;
                came_from[nkey] = key(current.x, current.y);
                open.push({nx, ny, tentative_g + heuristic(nx, ny, gx, gy)});
            }
        }
    }

    std::vector<Point2D> path;
    if (!found) return path; // empty: no plan found

    std::vector<int> cell_path;
    int cursor = key(gx, gy);
    cell_path.push_back(cursor);
    while (cursor != key(sx, sy)) {
        cursor = came_from.at(cursor);
        cell_path.push_back(cursor);
    }
    std::reverse(cell_path.begin(), cell_path.end());

    for (int k : cell_path) {
        const int cx = k % costmap_.width();
        const int cy = k / costmap_.width();
        double wx, wy;
        costmap_.gridToWorld(cx, cy, wx, wy);
        path.push_back({wx, wy});
    }
    return path;
}

// ---------------------------------------------------------------------
// HolonomicDWA
// ---------------------------------------------------------------------

HolonomicDWA::HolonomicDWA(Limits limits) : limits_(limits) {}

bool HolonomicDWA::simulateForClearance(const HolonomicVelocity& cmd, const Pose2D& current_pose,
                                         const Costmap2D& costmap, double& min_clearance_cost) const {
    Pose2D sim_pose = current_pose;
    min_clearance_cost = 0.0;
    const int steps = static_cast<int>(limits_.sim_horizon_s / limits_.time_step_s);

    for (int i = 0; i < steps; ++i) {
        const double c = std::cos(sim_pose.theta);
        const double s = std::sin(sim_pose.theta);
        sim_pose.x += (cmd.vx * c - cmd.vy * s) * limits_.time_step_s;
        sim_pose.y += (cmd.vx * s + cmd.vy * c) * limits_.time_step_s;
        sim_pose.theta += cmd.omega * limits_.time_step_s;

        int cx, cy;
        costmap.worldToGrid(sim_pose.x, sim_pose.y, cx, cy);
        if (costmap.isLethal(cx, cy)) {
            return false; // this candidate trajectory collides -- reject it
        }
        min_clearance_cost = std::max(min_clearance_cost, costmap.cost(cx, cy) / 255.0);
    }
    return true;
}

double HolonomicDWA::scoreCandidate(const HolonomicVelocity& cmd, const Pose2D& current_pose,
                                     const Point2D& next_waypoint, const Costmap2D& costmap) const {
    double clearance_cost = 0.0;
    if (!simulateForClearance(cmd, current_pose, costmap, clearance_cost)) {
        return -std::numeric_limits<double>::infinity(); // collision -> reject
    }

    // Project the final simulated position forward analytically to score
    // goal progress without re-simulating.
    const double c = std::cos(current_pose.theta);
    const double s = std::sin(current_pose.theta);
    const double final_x = current_pose.x + (cmd.vx * c - cmd.vy * s) * limits_.sim_horizon_s;
    const double final_y = current_pose.y + (cmd.vx * s + cmd.vy * c) * limits_.sim_horizon_s;

    const double dist_to_goal = std::hypot(next_waypoint.x - final_x, next_waypoint.y - final_y);
    const double goal_progress_score = -dist_to_goal; // closer is better

    const double speed = std::hypot(cmd.vx, cmd.vy);
    const double velocity_score = speed; // prefer actual progress over dithering

    const double obstacle_score = -clearance_cost; // lower inflation cost is better

    // Weighted sum -- weights are tunable hyperparameters in a real
    // system, typically chosen via simulation sweeps.
    constexpr double w_goal = 1.6, w_vel = 0.3, w_obstacle = 1.0;
    return w_goal * goal_progress_score + w_vel * velocity_score + w_obstacle * obstacle_score;
}

HolonomicVelocity HolonomicDWA::computeVelocity(const Pose2D& current_pose,
                                                  const HolonomicVelocity& current_velocity,
                                                  const Point2D& next_waypoint,
                                                  const Costmap2D& costmap) const {
    // Dynamic window: velocities reachable within one control cycle given
    // the robot's max acceleration.
    const double dt = limits_.time_step_s;
    const double vx_min = std::max(-limits_.max_vx, current_velocity.vx - limits_.max_accel_lin * dt);
    const double vx_max = std::min(limits_.max_vx, current_velocity.vx + limits_.max_accel_lin * dt);
    const double vy_min = std::max(-limits_.max_vy, current_velocity.vy - limits_.max_accel_lin * dt);
    const double vy_max = std::min(limits_.max_vy, current_velocity.vy + limits_.max_accel_lin * dt);
    const double w_min = std::max(-limits_.max_omega, current_velocity.omega - limits_.max_accel_ang * dt);
    const double w_max = std::min(limits_.max_omega, current_velocity.omega + limits_.max_accel_ang * dt);

    HolonomicVelocity best_cmd{};
    double best_score = -std::numeric_limits<double>::infinity();

    const int n = std::max(1, limits_.velocity_samples);
    for (int i = 0; i < n; ++i) {
        const double vx = vx_min + (vx_max - vx_min) * i / std::max(1, n - 1);
        for (int j = 0; j < n; ++j) {
            const double vy = vy_min + (vy_max - vy_min) * j / std::max(1, n - 1);
            for (int k = 0; k < n; ++k) {
                const double w = w_min + (w_max - w_min) * k / std::max(1, n - 1);

                const HolonomicVelocity candidate{vx, vy, w};
                const double score = scoreCandidate(candidate, current_pose, next_waypoint, costmap);
                if (score > best_score) {
                    best_score = score;
                    best_cmd = candidate;
                }
            }
        }
    }
    return best_cmd;
}

} // namespace smorphi
