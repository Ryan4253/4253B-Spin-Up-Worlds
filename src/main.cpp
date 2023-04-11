#include "main.h"
#include "autoSelect/selection.h"

void initialize() {
    // Auton Selector
    pros::lcd::initialize();
    // selector::init(); 

    imu->calibrate();
    superstructure->startTask();

    turnPID->setIntegratorReset(true);
    turnPID->setIntegralLimits(0.5, -0.5);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    leftChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    // switch (selector::auton) {
    //     case 0:
    //         Autons::skills();
    //         break;

    //     case 1:
    //         Autons::doNothing();
    //         break;

    //     case 2:
    //         Autons::simple();
    //         break;

    //     case 3:
    //         Autons::jonathan();
    //         break;

    //     case -1:
    //         Autons::doNothing();
    //         break;

    //     case -2:
    //         Autons::simple();
    //         break;

    //     case -3:
    //         Autons::jonathan();
    //         break;
    // }
}

void opcontrol() {
    std::vector<squiggles::ProfilePoint> path = squiggward->generate({{0, 0, 0}, {2, 1, 0}});
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

    leftChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    profiler->setTarget(path, {0, 0, 0}, true, true);
    while(true) pros::delay(50);

    // pointTurnToAngle(90_deg);

    leftChassis->setBrakeMode(AbstractMotor::brakeMode::coast);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::coast);

    auto model = std::static_pointer_cast<SkidSteerModel>(chassis->getModel());
    while(true) {
        model->curvature(
            master->getAnalog(ControllerAnalog::leftY), 
            master->getAnalog(ControllerAnalog::rightX),
            DEADBAND
        );
        // std::cout << "angle: " << chassis->getState().theta.convert(degree) << std::endl;
        pros::lcd::print(1, "X: %f    Y: %f", chassis->getState().x.convert(meter), chassis->getState().y.convert(meter));
        pros::lcd::print(2, "Odometry Angle: %f", lib4253::Math::angleWrap360(chassis->getState().theta.convert(degree)));
        pros::lcd::print(3, "IMU Angle: %f", imu->get());
        if(master->getDigital(ControllerDigital::A)) {
            superstructure->jog(0);
            superstructure->setPistonState(PistonState::PUNCHER_LOCK);
        } else if(master->getDigital(ControllerDigital::B)) {
            superstructure->jog(-1);
        } else if(master->getDigital(ControllerDigital::X)) {
            superstructure->jog(1);
        } else if(master->getDigital(ControllerDigital::Y)) {
            superstructure->jog(0);
            superstructure->setPistonState(PistonState::PUNCHER_UNLOCK);
        } else {
            superstructure->jog(0);
            intakeSolenoid->set(false);
        }
        pros::delay(10);
    }
}