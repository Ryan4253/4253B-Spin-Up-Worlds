#include "subsystems/superstructure.hpp"

Superstructure::Superstructure(const std::shared_ptr<okapi::MotorGroup> &imotors,
                               const std::shared_ptr<ryan::Solenoid> &ichassisSolenoid, 
                               const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid, 
                               const std::shared_ptr<okapi::ADIButton> &ipuncherLimitSwitch)
{
    motors = std::move(imotors);
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

void Superstructure::intake() {
    controlState = ControlState::AUTOMATIC;
    wantToIntake = true;
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
                } else {
                    chassisSolenoid->set(false);
                    if(pistonState == PistonState::INTAKE) {
                        puncherSolenoid->set(false);
                    } else {
                        puncherSolenoid->set(true);
                    }
                }
                motors->moveVoltage(jogSpeed * 12000);
                break;

            case ControlState::AUTOMATIC:
                if (!puncherLimitSwitch->isPressed()) {
                    fired = false;
                }
                if ((puncherLimitSwitch->isPressed() && fired) || (!puncherLimitSwitch->isPressed())) {
                    superstructureState = SuperstructureState::LOADING;
                    motors->moveVoltage(12000 * puncherSpeed);
                } else {
                    superstructureState = SuperstructureState::LOADED;
                    motors->moveVoltage(0);
                }
                if(wantToIntake && superstructureState == SuperstructureState::LOADED) {
                    superstructureState = SuperstructureState::INTAKING;
                    motors->moveVoltage(-12000 * intakeSpeed);
                }
                if(superstructureState == SuperstructureState::LOADED) {
                    chassisSolenoid->set(false);
                } else {
                    chassisSolenoid->set(true);
                }
                break;
        }
        pros::delay(10);
    }
}