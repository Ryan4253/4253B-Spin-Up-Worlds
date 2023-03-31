#pragma once

#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/button/adiButton.hpp"

#include "ryanlib/StateMachine.hpp"
#include "ryanlib/TaskWrapper.hpp"
#include "ryanlib/Solenoid.hpp"

enum class SuperstructureState { INTAKING, LOADED, LOADING, IDLE };

enum class ControlState { MANUAL, AUTOMATIC };

enum class PistonState { DISENGAGED, INTAKE, PUNCHER };

template class ryan::StateMachine<SuperstructureState>;

class Superstructure : public ryan::TaskWrapper, public ryan::StateMachine<SuperstructureState> {
    public:
    Superstructure(const std::shared_ptr<okapi::MotorGroup> &imotors,
                   const std::shared_ptr<ryan::Solenoid> &ichassisSolenoid, 
                   const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid, 
                   const std::shared_ptr<okapi::ADIButton> &ipuncherLimitSwitch);

    void disable(bool idisabled);
    void jog(double ipercentSpeed, PistonState ipistonState);
    void fire();
    void intake();
    void setPuncherSpeed(double ispeed);
    void setIntakeSpeed(double ispeed);

    void loop() override;

    protected:
    std::shared_ptr<okapi::MotorGroup> motors;
    std::shared_ptr<ryan::Solenoid> chassisSolenoid;
    std::shared_ptr<ryan::Solenoid> puncherSolenoid;
    std::shared_ptr<okapi::ADIButton> puncherLimitSwitch;

    private:
    bool isDisabled, fired, wantToIntake;
    double jogSpeed{0.0};
    double puncherSpeed{0.0};
    double intakeSpeed{0.0};
    ControlState controlState;
    PistonState pistonState;
    SuperstructureState superstructureState;
};