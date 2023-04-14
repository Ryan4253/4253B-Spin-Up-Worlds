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
}

bool Superstructure::isLoaded() const{
    return !puncherSolenoid->getState();
}

bool Superstructure::isDrive() const{
    return chassisSolenoid->getState();
}

bool Superstructure::isSuperstructure() const{
    return !chassisSolenoid->getState();
}

bool Superstructure::isPulledBack() const{
    return puncherEncoder->get() > 960;
}

void Superstructure::lockPuncher(){
    puncherSolenoid->set(false);
}

void Superstructure::unlockPuncher(){
    puncherSolenoid->set(true);
}

void Superstructure::setDriveMode(){
    chassisSolenoid->set(true);
}

void Superstructure::setSuperStructureMode(){
    chassisSolenoid->set(false);
}

void Superstructure::setDrive(double iLeftPower, double iRightPower){
    setDriveMode();
    leftMotor->moveVoltage(iLeftPower);
    rightMotor->moveVoltage(iRightPower);
}

void Superstructure::setIntake(uint16_t iPower){
    if(!isLoaded()){
        return;
    }
    setSuperStructureMode();
    leftMotor->moveVoltage(-iPower);
    rightMotor->moveVoltage(-iPower);
}

void Superstructure::setPuncher(uint16_t iPower){
    setSuperStructureMode();
    leftMotor->moveVoltage(iPower);
    rightMotor->moveVoltage(iPower);
}

void Superstructure::setDisable(bool iDisable){
    isDisabled = iDisable;
}

void Superstructure::shoot(){
    if(!isLoaded()){
        return;
    }
    unlockPuncher();
    pros::delay(50);
}

void Superstructure::loop(){
    while(true){
        pros::delay(10);
        if(isDisabled){
            continue;
        }

        if(isDrive()){
            continue;
        }

        if(isLoaded()){
            continue;
        }

        setPuncher(12000);
        if(isPulledBack()){
            setPuncher(0);
            puncherSolenoid->set(false);
        }
    }
}
