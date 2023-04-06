#include "main.h"
#include "autoSelect/selection.h"

void initialize() {
    // Auton Selector
    selector::init(); 

    imu->calibrate();
    superstructure->startTask();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    leftChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    switch (selector::auton) {
        case 0:
            Autons::skills();
            break;

        case 1:
            Autons::doNothing();
            break;

        case 2:
            Autons::simple();
            break;

        case 3:
            Autons::jonathan();
            break;

        case -1:
            Autons::doNothing();
            break;

        case -2:
            Autons::simple();
            break;

        case -3:
            Autons::jonathan();
            break;
    }
}

void opcontrol() {
    // std::vector<squiggles::ProfilePoint> path = squiggward->generate({{0, 0, 0}, {2, 0, 0}});
    // double prevVel = 0.0;
    // double prevAccel = 0.0;
    // for(int i = 0; i < path.size(); i++) {
    //     double vel = (path[i].wheel_velocities[0] + path[i].wheel_velocities[1]) / 2;
    //     double accel = (vel - prevVel) * 100;
    //     double jerk = (accel - prevAccel) * 100;
    //     prevVel = vel;
    //     prevAccel = accel;
    //     // std::cout << "Vel: " << vel << "    Accel: " << accel << "    Jerk: " << jerk << std::endl;
    //     std::cout << "(" << (double)i/100 << "," << accel << ")\n";
    //     pros::delay(10);
    // }

    // profiler->setTarget(path, {0, 0, 0}, false, true);
    // profiler->setTarget(1_ft, true);

    // squiggles::Constraints constraints = squiggles::Constraints(1, 1, 1);
    // std::unique_ptr<squiggles::SplineGenerator> squiggward = std::make_unique<squiggles::SplineGenerator>(
    //     constraints, 
    //     std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(okapi::meter), constraints), 
    //     0.01
    // );
    // std::vector<squiggles::Pose> iPathPoints = {{0, 0, 0}, {0, 1, 0}};
    // squiggward->generate(iPathPoints);
    // std::cout << "done :)\n";

    leftChassis->setBrakeMode(AbstractMotor::brakeMode::coast);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::coast);

    auto model = std::static_pointer_cast<SkidSteerModel>(chassis->getModel());
    while(true) {
        model->curvature(
            master->getAnalog(ControllerAnalog::leftY), 
            master->getAnalog(ControllerAnalog::rightX),
            DEADBAND
        );
        pros::delay(10);
    }
}