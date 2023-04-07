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
#include "ryanlib/AsyncOdomProfiler.hpp"

#include "subsystems/superstructure.hpp"

#define LVGL_SCREEN_WIDTH 480
#define LVGL_SCREEN_HEIGHT 240

using namespace okapi;

// CONSTANTS
const double DEADBAND = 0.0500;

// CONTROLLER(s)
extern std::shared_ptr<Controller> master;

// MOTORS
extern std::shared_ptr<MotorGroup> leftChassis;
extern std::shared_ptr<MotorGroup> rightChassis;

extern std::shared_ptr<MotorGroup> superstructureMotors;

// PNEUMAICS
extern std::shared_ptr<ryan::Solenoid> intakeSolenoid; 

// SENSORS
extern std::shared_ptr<IMU> imu;

// SUBSYSTEM CONTROLLERS
extern std::shared_ptr<OdomChassisController> chassis;
extern std::shared_ptr<ryan::AsyncOdomMotionProfiler> profiler;
extern std::shared_ptr<IterativePosPIDController> turnPID;
extern std::shared_ptr<Superstructure> superstructure;
extern std::shared_ptr<squiggles::SplineGenerator> squiggward;