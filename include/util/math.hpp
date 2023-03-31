#pragma once
#include "util/point.hpp"

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/units/QSpeed.hpp"

#include <memory>
#include <optional>
#include <cmath>
#include <math.h>

namespace lib4253::Math{

using namespace okapi;

QLength angleToArcLength(QAngle angle, QLength wheelRadius);

QAngle arcLengthToAngle(QLength dist, QLength wheelRadius);

double velToRPM(QSpeed vel, QLength wheelRadius, double gearRatio = 1);

QSpeed rpmToVel(double rpm, QLength wheelRadius, double gearRatio = 1);

double angleWrap360(double angle);

double angleWrapp180(double angle);

double angleWrap90(double angle);

QAngle angleWrap360(QAngle angle);

QAngle angleWrap180(QAngle angle);

QAngle angleWrap90(QAngle angle);

double sinc(double x);

QLength circumradius(const Translation& side, const Translation& mid, const Translation& right);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

};