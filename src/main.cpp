#include "main.h"
#include "autoSelect/selection.h"

void initialize() {
    // Auton Selector
    pros::lcd::initialize();
    // selector::init(); 

    imu->calibrate();
    puncherEncoder->reset();
    superstructure->startTask();

    pointTurnPID->setIntegratorReset(true);
    pointTurnPID->setIntegralLimits(0.5, -0.5);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    leftChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    rightChassis->setBrakeMode(AbstractMotor::brakeMode::brake);
    Autons::awp();
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
    // auto path = squiggward->generate({{0, 0, 0}, {0.53, -0.32, -45 * degreeToRadian}});
    // profiler->setTarget(path, {0, 0, 0}, true, true);

    // pivotTurnToAngle(-45_deg, ChassisSide::RIGHT);
    Autons::jonathan();


    while(true) {
        std::cout << puncherEncoder->get() << std::endl;
        pros::lcd::print(0, "X: %f    Y: %f", chassis->getState().x.convert(foot), chassis->getState().y.convert(foot));
        pros::lcd::print(1, "Theta: %f", chassis->getState().theta.convert(degree));
        pros::lcd::print(2, "IMU Angle: %f", imu->get());
        
        auto [leftPower, rightPower] = curvatureDrive(
            master->getAnalog(ControllerAnalog::leftY), 
            master->getAnalog(ControllerAnalog::rightX),
            DEADBAND
        );

        chassis->getModel()->tank(leftPower, rightPower);

        if(master->getDigital(ControllerDigital::R2)){
            superstructure->setDrive(leftPower, rightPower);
        }
        else{
            superstructure->setIntake(master->getDigital(ControllerDigital::L1) * 12000);
        }
        
        if(master->getDigital(ControllerDigital::R1)) {
            superstructure->shoot();
        } 

        expansionSolenoid->set(master->getDigital(ControllerDigital::A));
        intakeSolenoid->set(master->getDigital(ControllerDigital::Y));
        bandReleaseSolenoid->set(master->getDigital(ControllerDigital::X));
    
        pros::delay(10);
    }
}