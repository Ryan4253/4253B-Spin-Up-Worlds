#pragma once

#include "okapi/api/units/QAngle.hpp"

#include "ryanlib/AsyncProfiler.hpp"
#include "ryanlib/Math.hpp"

#include "../globals.hpp"

enum class ChassisSide {
    LEFT, RIGHT
};

void pointTurnToAngle(okapi::QAngle targetAngle);

void pivotTurnToAngle(okapi::QAngle targetAngle, double leftScaler, double rightScaler);

void pivotTurnToAngle(okapi::QAngle targetAngle, ChassisSide drivingSide);