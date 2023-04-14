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
    while(true) {
        
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
            superstructure->setIntake(master->getDigital(ControllerDigital::L1)*12000);
        }
        
        if(master->getDigital(ControllerDigital::R1)) {
            superstructure->shoot();
        } 

        expansionSolenoid->set(master->getDigital(ControllerDigital::A));
        intakeSolenoid->set(master->getDigital(ControllerDigital::Y));
    
        pros::delay(10);
    }
}