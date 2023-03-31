#include "globals.hpp"

// CONTROLLERS
auto master = std::make_shared<Controller>(okapi::ControllerId::master);

// MOTORS
Motor leftFront(10, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftMid(9, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFront(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightMid(6, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(5, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor leftSuperstructure(1, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightSuperstructure(2, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

auto leftChassis = std::make_shared<MotorGroup>(leftFront, leftMid, leftBack);
auto rightChassis = std::make_shared<MotorGroup>(rightFront, rightMid, rightBack);

auto superstructureMotors = std::make_shared<MotorGroup>(leftSuperstructure, rightSuperstructure);

// SOLENOIDS
auto chassisSolenoid = std::make_shared<ryan::Solenoid>('A');
auto puncherSolenoid = std::make_shared<ryan::Solenoid>('B');
auto expansionSolenoid = std::make_shared<ryan::Solenoid>('G');

// SENSORS
auto imu = std::make_shared<IMU>(20);
auto puncherLimitSwitch = std::make_shared<ADIButton>('C');

// MOTION PROFILE CONSTANTS
ryan::ProfileConstraint moveLimit({6_ftps, 10_ftps2, 10_ftps2, 34_ftps3}); //! todo!

// SUBSYSTEM CONTROLLERS
auto chassis = ChassisControllerBuilder()
    .withMotors(leftChassis, rightChassis)
    .withDimensions(
        {AbstractMotor::gearset::blue, 1.0}, 
        {{2.75_in, 1.294_ft}, imev5BlueTPR}
    )
    .build();

auto profiler = ryan::AsyncMotionProfilerBuilder()
    .withOutput(chassis)
    .withProfiler(std::make_unique<ryan::SCurveMotionProfile>(moveLimit))
    .build();

auto turnPID = std::make_shared<IterativePosPIDController>
    (0.037, 0.0, 0.00065, 0, TimeUtilFactory::withSettledUtilParams(1, 2, 100_ms));

auto superstructure = std::make_shared<Superstructure>
    (superstructureMotors, chassisSolenoid, puncherSolenoid, puncherLimitSwitch);