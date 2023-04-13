#include "subsystems/superstructure.hpp"

Superstructure::Superstructure(const std::shared_ptr<okapi::Motor> &ileftMotor,
                               const std::shared_ptr<okapi::Motor> &irightMotor,
                               const std::shared_ptr<ryan::Solenoid> &ichassisSolenoid, 
                               const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid, 
                               const std::shared_ptr<okapi::RotationSensor> &ipuncherEncoder)
{
    leftMotor = std::move(ileftMotor);
    rightMotor = std::move(irightMotor);
    chassisSolenoid = std::move(ichassisSolenoid);
    puncherSolenoid = std::move(ipuncherSolenoid);
    puncherEncoder = std::move(ipuncherEncoder);
    puncherEncoder->reset();

    isDisabled = false;
    fired = false;
    pistonState = PistonState::DISENGAGED;
    controlState = ControlState::MANUAL;
}

void Superstructure::disable(bool idisabled) {
    isDisabled = idisabled;
}

void Superstructure::jog(double ipercentSpeed) {
    controlState = ControlState::MANUAL;
    jogSpeed = ipercentSpeed;
}

void Superstructure::setPistonState(PistonState ipistonState) {
    controlState = ControlState::MANUAL;
    pistonState = ipistonState;
}

void Superstructure::fire(bool ifirstTime) {
    controlState = ControlState::AUTOMATIC;
    fired = true;
    wantToIntake = false;
    loaded = false;
    firstTime = ifirstTime;
}

void Superstructure::intake(bool iwantToIntake) {
    controlState = ControlState::AUTOMATIC;
    wantToIntake = iwantToIntake;
}

void Superstructure::drive(bool iwantToDrive, double ileftSpeed, double irightSpeed) {
    controlState = ControlState::AUTOMATIC;
    wantToDrive = iwantToDrive;
    leftSpeed = ileftSpeed;
    rightSpeed = irightSpeed;
}

void Superstructure::setPuncherSpeed(double ispeed) {
    puncherSpeed = std::abs(ispeed);
}

void Superstructure::setIntakeSpeed(double ispeed) {
    intakeSpeed = std::abs(ispeed);
}

void Superstructure::loop() {
    if(isDisabled) return;
    
    while (true) {
        if(controlState == ControlState::MANUAL) {
            leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
            rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
            if(pistonState == PistonState::DISENGAGED) {
                chassisSolenoid->set(true);
            } else {
                chassisSolenoid->set(false);
                if(pistonState == PistonState::PUNCHER_UNLOCK) {
                    puncherSolenoid->set(true);
                } else {
                    puncherSolenoid->set(false);
                }
            }
            leftMotor->moveVoltage(jogSpeed * 12000);
            rightMotor->moveVoltage(jogSpeed * 12000);
            // std::cout << "L: " << leftMotor->getEfficiency() << "R: " << rightMotor->getEfficiency() << std::endl;
        } else if(controlState == ControlState::AUTOMATIC) {
            driving = false;

            
            if(!loaded && puncherEncoder->get() < 940) {
                leftMotor->moveVoltage(puncherSpeed * 12000);
                rightMotor->moveVoltage(puncherSpeed * 12000);
                puncherSolenoid->set(true);
            } else if(loaded && wantToIntake) {
                leftMotor->moveVoltage(intakeSpeed * -12000);
                rightMotor->moveVoltage(intakeSpeed * -12000);
                puncherSolenoid->set(false);
            } else if(loaded && fired) {
                puncherSolenoid->set(true);
            } else {
                leftMotor->moveVoltage(0);
                rightMotor->moveVoltage(0);
                puncherSolenoid->set(false);
            }

            if(puncherEncoder->get() > 940) { 
                loaded = true;
            }




            // if(wantToIntake && puncherEncoder->get() > 940) {
            //     wantToDrive = false;
            //     leftMotor->moveVoltage(intakeSpeed * -12000);
            //     rightMotor->moveVoltage(intakeSpeed * -12000);
            // } 
            // if(!wantToIntake && puncherEncoder->get() < 940) {
            //     wantToDrive = false;
            //     leftMotor->moveVoltage(puncherSpeed * 12000);
            //     rightMotor->moveVoltage(puncherSpeed * 12000);
            //     puncherSolenoid->set(true);
            // } else  {
            //     leftMotor->moveVoltage(0);
            //     rightMotor->moveVoltage(0);
            //     puncherSolenoid->set(false);
            //     if(wantToIntake) {
            //         leftMotor->moveVoltage(intakeSpeed * -12000);
            //         rightMotor->moveVoltage(intakeSpeed * -12000);
            //     }
            // }
            // if(!wantToIntake && puncherEncoder->get() > 940 && fired) {
            //     wantToDrive = false;
            //     puncherSolenoid->set(true);
            //     pros::delay(1000);
            // }



            // if(wantToDrive) {
            //     driving = true;
            //     leftMotor->moveVoltage(leftSpeed * 12000);
            //     rightMotor->moveVoltage(rightSpeed * 12000);
            // }
            // if(driving) {
            //     chassisSolenoid->set(true);
            //     leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
            //     rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
            // } else {
            //     chassisSolenoid->set(false);
            //     leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
            //     rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
            // }
        }
        pros::delay(10);
    }
}