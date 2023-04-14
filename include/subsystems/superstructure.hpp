#pragma once

#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/button/adiButton.hpp"
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"

#include "ryanlib/StateMachine.hpp"
#include "ryanlib/TaskWrapper.hpp"
#include "ryanlib/Solenoid.hpp"

class Superstructure : public ryan::TaskWrapper{
    public:
    Superstructure(const std::shared_ptr<okapi::Motor> &ileftMotor,
                   const std::shared_ptr<okapi::Motor> &irightMotor,
                   const std::shared_ptr<ryan::Solenoid> &ichassisSolenoid, 
                   const std::shared_ptr<ryan::Solenoid> &ipuncherSolenoid,
                   const std::shared_ptr<okapi::RotationSensor> &ipuncherEncoder);

    bool isLoaded() const;

    bool isDrive() const;

    bool isSuperstructure() const;

    bool isPulledBack() const;

    void lockPuncher();

    void unlockPuncher();

    void setDriveMode();

    void setSuperStructureMode();

    void setDrive(double iLeftPower, double iRightPower);

    void setIntake(uint16_t iPower);

    void setPuncher(uint16_t iPower);

    void setDisable(bool iDisable);

    void shoot();

    void loop() override;
    

    protected:
    std::shared_ptr<okapi::Motor> leftMotor, rightMotor;
    std::shared_ptr<ryan::Solenoid> chassisSolenoid;
    std::shared_ptr<ryan::Solenoid> puncherSolenoid;
    std::shared_ptr<okapi::RotationSensor> puncherEncoder;

    private:
    bool isDisabled;
};