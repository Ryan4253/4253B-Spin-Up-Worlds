#pragma once
#include "okapi/api/units/QAngle.hpp"
#include "ryanlib/AsyncProfiler.hpp"

enum class ChassisSide {
    LEFT, RIGHT
};

class Chassis : public ryan::AsyncMotionProfiler {
    
};

void pointTurnToAngle(okapi::QAngle targetAngle);

void pivotTurnToAngle(okapi::QAngle targetAngle, double leftScaler, double rightScaler);

void pivotTurnToAngle(okapi::QAngle targetAngle, ChassisSide drivingSide);