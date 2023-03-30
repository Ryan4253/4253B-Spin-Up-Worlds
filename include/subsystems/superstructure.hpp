#pragma once
#include "ryanlib/StateMachine.hpp"
#include "ryanlib/TaskWrapper.hpp"
#include "ryanlib/Solenoid.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"

enum class SuperstructureState { INTAKING, ROLLING, LOAD_POSITION, LOADING, FIRING, IDLE };

enum class ControlState { MANUAL, AUTOMATIC };

enum class PistonState { DISENGAGED, INTAKE, PUNCHER };

template class ryan::StateMachine<ControlState>;

class Superstructure : public ryan::TaskWrapper, public ryan::StateMachine<ControlState> {
    public:
    Superstructure(const std::shared_ptr<okapi::MotorGroup> &imotors,
                   const std::shared_ptr<ryan::Solenoid> &idriveSolenoid, 
                   const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid);

    void disable(bool idisabled);
    void jog(double ipercentSpeed, PistonState ipistonState);

    void fire();

    ControlState getState();
    void loop() override;

    protected:
    std::shared_ptr<okapi::MotorGroup> motors;
    std::shared_ptr<ryan::Solenoid> driveSolenoid;
    std::shared_ptr<ryan::Solenoid> puncherSolenoid;

    private:
    bool isDisabled;
    double jogSpeed{0.0};
    ControlState state;
    PistonState pistonState;
};