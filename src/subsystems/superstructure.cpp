#include "subsystems/superstructure.hpp"

Superstructure::Superstructure(const std::shared_ptr<okapi::MotorGroup> &imotors,
                               const std::shared_ptr<ryan::Solenoid> &idriveSolenoid, 
                               const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid)
{
    motors = std::move(imotors);
    driveSolenoid = std::move(idriveSolenoid);
    puncherSolenoid = std::move(ipuncherSolenoid);
    isDisabled = false;
    pistonState = PistonState::DISENGAGED;
    state = ControlState::MANUAL;
}

void Superstructure::disable(bool idisabled) {
    isDisabled = idisabled;
}

void Superstructure::jog(double ipercentSpeed, PistonState ipistonState) {
    state = ControlState::MANUAL;
    jogSpeed = ipercentSpeed;
    pistonState = ipistonState;
}

void Superstructure::loop() {
    if(isDisabled) return;
    
    while (true) {
        switch (state) {
            case ControlState::MANUAL:
                if(pistonState == PistonState::DISENGAGED) {
                    driveSolenoid->set(true);
                } else {
                    driveSolenoid->set(false);
                }
                if(pistonState == PistonState::INTAKE) {
                    puncherSolenoid->set(false);
                } else {
                    puncherSolenoid->set(true);
                }
                motors->moveVoltage(jogSpeed * 12000);
                break;

            case ControlState::AUTOMATIC:
                if (!button->isPressed()) {
                    fired = false;
                }
                if ((button->isPressed() && fired) || (!button->isPressed())) {
                    motor->moveVoltage(12000 * cataSpeed);
                    cataState = CatapultState::MOVING;
                } else {
                    motor->moveVoltage(0);
                    cataState = CatapultState::LOAD_POSITION;
                }
                break;
        }
        pros::delay(10);
    }
}
