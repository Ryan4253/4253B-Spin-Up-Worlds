#include "main.h"

void initialize() {
    pros::lcd::initialize();

    // Auton Selector
    const char *autons[3] = {"a", "b", "c"};
    Selector::init(180, 1, autons);

    imu->calibrate();
    // catapult->startTask();
}

void disabled() {}

void competition_initialize() {}
/*
void autonomous() {
    leftChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    switch (Selector::auton) {
        case 0:
            // doNothing();
            break;

        case 1:
            autonA();
            break;

        case 2:
            // redAutonB();
            break;

        case 3:
            // redAutonC();
            break;

        case -1:
            // blueAutonA();
            break;

        case -2:
            // blueAutonB();
            break;

        case -3:
            // blueAutonC();
            break;
        }
}

void opcontrol() {
    profiler->setTarget({{0, 0, 0}, {0, 1, 0}}, false, true);

    // leftChassis->setBrakeMode(AbstractMotor::brakeMode::coast);
    // rightChassis->setBrakeMode(AbstractMotor::brakeMode::coast);

    // auto model = std::static_pointer_cast<SkidSteerModel>(chassis->getModel());
    // while(true) {
    //     model->curvature(
    //         master->getAnalog(ControllerAnalog::leftY), 
    //         master->getAnalog(ControllerAnalog::rightX),
    //         DEADBAND
    //     );
    //     pros::delay(10);
    // }
}
*/