#include "squiggles/include/squiggles.hpp"
#include <iostream>

namespace sq = squiggles;

int main() {
    const double MAX_VEL = 2.0;     // in meters per second
    const double MAX_ACCEL = 3.0;   // in meters per second per second
    const double MAX_JERK = 6.0;    // in meters per second per second per second
    const double ROBOT_WIDTH = 0.4; // in meters
    auto constraints = sq::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
    auto generator = sq::SplineGenerator(
        constraints,
        std::make_shared<sq::TankModel>(ROBOT_WIDTH, constraints)
    );

    auto path = generator.generate({sq::Pose(0, 0, 0), sq::Pose(2, 2, 0)});

    for(int i = 0; i < path.size(); i++) {
        std::cout << "Left: " << path[i].wheel_velocities[0] << "    Right: " << path[i].wheel_velocities[1] << std::endl;
    }

    return 0;
}