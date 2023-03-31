#include "subsystems/chassis.hpp"

void pointTurnToAngle(okapi::QAngle targetAngle) {
    turnPID->reset();
    turnPID->setTarget(0);

    do {
        chassis->getModel()->arcade(
          0, turnPID->step(-ryan::Math::rescale180(targetAngle.convert(degree) - imu->get())));
        pros::delay(10);
    } while (!turnPID->isSettled());

    (chassis->getModel())->stop();
}

void pivotTurnToAngle(okapi::QAngle targetAngle, double leftScaler, double rightScaler) {
    turnPID->reset();
    turnPID->setTarget(0);

    do {
        double stepVal = turnPID->step(-ryan::Math::rescale180(targetAngle.convert(degree) - imu->get()));
        if(stepVal > 1) stepVal = 1;
        double left = stepVal * leftScaler;
        double right = stepVal * rightScaler;

        chassis->getModel()->tank(left, right);
        pros::delay(10);
    } while (!turnPID->isSettled());

    (chassis->getModel())->stop();
}

void pivotTurnToAngle(okapi::QAngle targetAngle, ChassisSide drivingSide) {
    turnPID->reset();
    turnPID->setTarget(0);

    do {
        double stepVal = turnPID->step(-ryan::Math::rescale180(targetAngle.convert(degree) - imu->get()));
        double left = drivingSide == ChassisSide::LEFT ? stepVal : 0;
        double right = drivingSide == ChassisSide::RIGHT ? stepVal : 0;

        chassis->getModel()->tank(left, right);
        pros::delay(10);
    } while (!turnPID->isSettled());

    (chassis->getModel())->stop();
}