#pragma once

#include "include/okapi/impl/device/motor/motor.hpp"
#include "include/okapi/impl/device/motor/motorGroup.hpp"
#include "include/okapi/impl/device/controller.hpp"
#include "include/okapi/impl/device/rotarysensor/IMU.hpp"
#include "include/okapi/api/chassis/controller/chassisController.hpp"
#include "include/okapi/api/control/iterative/iterativePosPidController.hpp"

#include "ryanlib/Solenoid.hpp"
#include "ryanlib/AsyncProfiler.hpp"

#define LVGL_SCREEN_WIDTH 480
#define LVGL_SCREEN_HEIGHT 240

// CONSTANTS
const double DEADBAND = 0.0500;

// CONTROLLER(s)
extern okapi::Controller master;

// MOTORS
extern okapi::MotorGroup leftChassis;
extern okapi::MotorGroup rightChassis;
// extern Motor catapultMotor;
extern okapi::Motor intake;

// PNEUMAICS
extern ryan::Solenoid intakeAngler;
extern ryan::Solenoid expansion;

// SENSORS
extern okapi::IMU imu;
// extern ADIButton catapultButton;

// SUBSYSTEM CONTROLLERS
extern std::shared_ptr<okapi::ChassisController> chassis;
extern std::shared_ptr<ryan::AsyncMotionProfiler> profiler;
extern std::shared_ptr<okapi::IterativePosPIDController> turnPID;