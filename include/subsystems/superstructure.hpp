#pragma once

#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/button/adiButton.hpp"

#include "ryanlib/StateMachine.hpp"
#include "ryanlib/TaskWrapper.hpp"
#include "ryanlib/Solenoid.hpp"

enum class SuperstructureState { LOADED, LOADING, IDLE };

enum class ControlState { MANUAL, AUTOMATIC };

enum class PistonState { DISENGAGED, PUNCHER_LOCK, PUNCHER_UNLOCK };

template class ryan::StateMachine<SuperstructureState>;

class Superstructure : public ryan::TaskWrapper, public ryan::StateMachine<SuperstructureState> {
    public:
    Superstructure(const std::shared_ptr<okapi::Motor> &ileftMotor,
                   const std::shared_ptr<okapi::Motor> &irightMotor,
                   const std::shared_ptr<ryan::Solenoid> &ichassisSolenoid, 
                   const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid, 
                   const std::shared_ptr<okapi::ADIButton> &ipuncherLimitSwitch);

    void disable(bool idisabled);
    void jog(double ipercentSpeed);
    void setPistonState(PistonState ipistonState);
    void fire();
    void intake(bool iwantToIntake);
    void drive(bool iwantToDrive, double ileftSpeed, double irightSpeed);
    void setPuncherSpeed(double ispeed);
    void setIntakeSpeed(double ispeed);
    void autonomousEnabled(bool iisEnabled);

    void loop() override;

    protected:
    std::shared_ptr<okapi::Motor> leftMotor, rightMotor;
    std::shared_ptr<ryan::Solenoid> chassisSolenoid;
    std::shared_ptr<ryan::Solenoid> puncherSolenoid;
    std::shared_ptr<okapi::ADIButton> puncherLimitSwitch;

    private:
    bool isDisabled, fired, wantToIntake, wantToDrive, isAutonomousEnabled, driving;
    double jogSpeed{0.0};
    double puncherSpeed{1.0};
    double intakeSpeed{1.0};
    double leftSpeed{0.0};
    double rightSpeed{0.0};
    ControlState controlState;
    PistonState pistonState;
    SuperstructureState superstructureState;
};