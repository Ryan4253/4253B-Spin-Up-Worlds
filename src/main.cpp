#include "main.h"
#include "autoSelect/selection.h"

double curve(double input){
    input = input * 100;
    const double t = 4.5/10;

    const double output = (pow(2.71828, -t) + 
                           pow(2.71828, (std::abs(input)-100)/10) * 
                           (1-pow(2.71818, -t))) * input; 

    return output / 100;
}

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
    Autons::rollerRight();
}

void opcontrol() {
    bool disableShooter = false;

    while(true) {
        std::cout << puncherEncoder->get() << std::endl;
        pros::lcd::print(0, "X: %f    Y: %f", chassis->getState().x.convert(foot), chassis->getState().y.convert(foot));
        pros::lcd::print(1, "Theta: %f", chassis->getState().theta.convert(degree));
        pros::lcd::print(2, "IMU Angle: %f", imu->get());
        
        auto [leftPower, rightPower] = tankDrive(
            curve(master->getAnalog(ControllerAnalog::leftY)),
            curve(master->getAnalog(ControllerAnalog::rightY)),
            DEADBAND
        );

        chassis->getModel()->tank(leftPower, rightPower);

        if(master->getDigital(ControllerDigital::R2)){
            superstructure->setDrive(leftPower, rightPower);
            master->rumble("... ");
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
    
        if(master->getDigital(ControllerDigital::B)){
            disableShooter = !disableShooter;
            superstructure->setDisable(disableShooter);
        }

        pros::delay(10);
    }
}