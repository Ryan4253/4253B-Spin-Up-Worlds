#pragma once

#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/controller.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/impl/device/button/adiButton.hpp"

#include "ryanlib/Solenoid.hpp"
#include "ryanlib/AsyncProfiler.hpp"

#define LVGL_SCREEN_WIDTH 480
#define LVGL_SCREEN_HEIGHT 240

using namespace okapi;

// CONSTANTS
const double DEADBAND = 0.0500;

// CONTROLLER(s)
extern Controller master;

// MOTORS
extern MotorGroup leftChassis;
extern MotorGroup rightChassis;

extern MotorGroup superstructure;

// PNEUMAICS
extern ryan::Solenoid chassisSolenoid;
extern ryan::Solenoid puncherSolenoid;
extern ryan::Solenoid expansionSolenoid;

// SENSORS
extern IMU imu;
extern ADIButton puncherButton;

// SUBSYSTEM CONTROLLERS
extern std::shared_ptr<ChassisController> chassis;
extern std::shared_ptr<ryan::AsyncMotionProfiler> profiler;
extern std::shared_ptr<IterativePosPIDController> turnPID;