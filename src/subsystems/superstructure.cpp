#include "subsystems/superstructure.hpp"

Superstructure::Superstructure(const std::shared_ptr<okapi::Motor> &ileftMotor,
                               const std::shared_ptr<okapi::Motor> &irightMotor,
                               const std::shared_ptr<ryan::Solenoid> &ichassisSolenoid, 
                               const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid, 
                               const std::shared_ptr<okapi::ADIButton> &ipuncherLimitSwitch)
{
    leftMotor = std::move(ileftMotor);
    rightMotor = std::move(irightMotor);
    chassisSolenoid = std::move(ichassisSolenoid);
    puncherSolenoid = std::move(ipuncherSolenoid);
    puncherLimitSwitch = std::move(ipuncherLimitSwitch);

    isDisabled = false;
    fired = false;
    pistonState = PistonState::DISENGAGED;
    controlState = ControlState::MANUAL;
}

void Superstructure::disable(bool idisabled) {
    isDisabled = idisabled;
}

void Superstructure::jog(double ipercentSpeed, PistonState ipistonState) {
    controlState = ControlState::MANUAL;
    jogSpeed = ipercentSpeed;
    pistonState = ipistonState;
}

void Superstructure::fire() {
    controlState = ControlState::AUTOMATIC;
    fired = true;
    wantToIntake = false;

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
        switch (controlState) {
            case ControlState::MANUAL:
                if(pistonState == PistonState::DISENGAGED) {
                    chassisSolenoid->set(true);
                    leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
                    rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
                } else {
                    chassisSolenoid->set(false);
                    leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
                    rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
                    if(pistonState == PistonState::INTAKE) {
                        puncherSolenoid->set(false);
                    } else {
                        puncherSolenoid->set(true);
                    }
                }
                leftMotor->moveVoltage(jogSpeed * 12000);
                rightMotor->moveVoltage(jogSpeed * 12000);
                break;

            case ControlState::AUTOMATIC:
                driving = false;
                if (!puncherLimitSwitch->isPressed()) {
                    fired = false;
                }
                if ((puncherLimitSwitch->isPressed() && fired) || (!puncherLimitSwitch->isPressed())) {
                    superstructureState = SuperstructureState::LOADING;
                    leftMotor->moveVoltage(puncherSpeed * 12000);
                    rightMotor->moveVoltage(puncherSpeed * 12000);
                } else {
                    superstructureState = SuperstructureState::LOADED;
                    leftMotor->moveVoltage(0);
                    rightMotor->moveVoltage(0);
                }
                if(wantToIntake && superstructureState != SuperstructureState::LOADING) {
                    leftMotor->moveVoltage(intakeSpeed * -12000);
                    rightMotor->moveVoltage(intakeSpeed * -12000);
                } else if(wantToDrive && superstructureState != SuperstructureState::LOADING) {
                    driving = true;
                    leftMotor->moveVoltage(leftSpeed * 12000);
                    rightMotor->moveVoltage(rightSpeed * 12000);
                }
                if(driving) {
                    chassisSolenoid->set(true);
                    leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
                    rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
                } else {
                    chassisSolenoid->set(false);
                    leftMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
                    rightMotor->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
                }
                break;
        }
        pros::delay(10);
    }
}