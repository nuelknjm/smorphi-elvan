#include "StateMachine.hpp"
#include <deque>
#include <memory>

int main() {
    using namespace smorphi;

    // Arena sized generously around the three target coordinates
    // (0,2) -> (2,3) -> (-2,4), in a world frame centered at the start (0,0).
    constexpr double resolution = 0.05;      // 5 cm / cell
    constexpr double origin_x = -4.0;        // world x of grid cell (0,0)
    constexpr double origin_y = -1.0;        // world y of grid cell (0,0)
    constexpr int width_cells = 160;         // 8 m across
    constexpr int height_cells = 120;        // 6 m tall
    constexpr double inflation_radius = 0.3; // meters

    Costmap2D costmap(width_cells, height_cells, resolution, origin_x, origin_y, inflation_radius);

    std::deque<Point2D> targets = {
        {0.0, 2.0},
        {2.0, 3.0},
        {-2.0, 4.0},
    };

    SmorphiNavigationFSM fsm(
        std::make_unique<SimulatedIMU>(),
        std::make_unique<SimulatedLidar>(),
        std::move(targets),
        std::move(costmap));

    fsm.run();
    return 0;
}
