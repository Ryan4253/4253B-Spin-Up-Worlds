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

void autonomous() {
    switch (Selector::auton) {
        case 0:
            // doNothing();
            break;

        case 1:
            // redAutonA();
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
