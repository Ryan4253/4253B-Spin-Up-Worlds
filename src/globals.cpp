#include "globals.hpp"

// CONTROLLERS
Controller master(okapi::ControllerId::master);

// MOTORS
Motor leftFront(10, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftMid(9, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFront(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightMid(6, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(5, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor leftSuperstructure(1, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightSuperstructure(2, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

MotorGroup leftChassis({leftFront, leftMid, leftBack});
MotorGroup rightChassis({rightFront, rightMid, rightBack});

MotorGroup superstructure({leftSuperstructure, rightSuperstructure});

// SOLENOIDS
ryan::Solenoid chassisSolenoid('A');
ryan::Solenoid puncherSolenoid('B');
ryan::Solenoid expansionSolenoid('G');

// SENSORS
IMU imu(20);
ADIButton puncherButton('C');

// MOTION PROFILE CONSTANTS
ryan::ProfileConstraint moveLimit({6_ftps, 10_ftps2, 10_ftps2, 34_ftps3}); //! todo!

// SUBSYSTEM CONTROLLERS
std::shared_ptr<ChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors(leftChassis, rightChassis)
    // !todo!
    .withDimensions({AbstractMotor::gearset::blue, 1.0}, {{2.75_in, 1.294_ft}, imev5BlueTPR})
    .build();

std::shared_ptr<ryan::AsyncMotionProfiler> profiler =
  ryan::AsyncMotionProfilerBuilder()
    .withOutput(chassis)
    .withProfiler(std::make_unique<ryan::SCurveMotionProfile>(moveLimit))
    .build();

std::shared_ptr<IterativePosPIDController> turnPID =
  std::make_shared<IterativePosPIDController>(0.037,
                                              0.0,
                                              0.00065,
                                              0,
                                              TimeUtilFactory::withSettledUtilParams(1, 2, 100_ms));