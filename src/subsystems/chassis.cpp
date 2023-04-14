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

std::pair<double, double> curvatureDrive(double throttle, double curvature, double deadband){
    throttle = std::clamp(throttle, -1.0, 1.0);
    if (std::abs(throttle) < deadband) {
        throttle = 0;
    }

    curvature = std::clamp(curvature, -1.0, 1.0);
    if (std::abs(curvature) < deadband) {
        curvature = 0;
    }

    // the algorithm switches to arcade when forward speed is 0 to allow point turns.
    double leftSpeed, rightSpeed;
    if (throttle == 0) {
        leftSpeed = throttle + curvature;
        rightSpeed = throttle - curvature;
    }
    else{
        leftSpeed = throttle + std::abs(throttle) * curvature;
        rightSpeed = throttle - std::abs(throttle) * curvature;
    }

    double maxSpeed = std::max(leftSpeed, rightSpeed);

    // normalizes output
    if (maxSpeed > 1.0) {
        leftSpeed /= maxSpeed;
        rightSpeed /= maxSpeed;
    }

    return std::make_pair(leftSpeed, rightSpeed);
}