#include "globals.hpp"

// CONTROLLERS
std::shared_ptr<Controller> master(new Controller(okapi::ControllerId::master));

// MOTORS
Motor leftFront(10, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftMid(9, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFront(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightMid(6, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(5, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor leftSuperstructure(1, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightSuperstructure(2, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<MotorGroup> leftChassis(new MotorGroup({leftFront, leftMid, leftBack}));
std::shared_ptr<MotorGroup> rightChassis(new MotorGroup({rightFront, rightMid, rightBack}));

std::shared_ptr<MotorGroup> superstructureMotors(new MotorGroup({leftSuperstructure, rightSuperstructure}));

// SOLENOIDS
std::shared_ptr<ryan::Solenoid> chassisSolenoid = std::make_shared<ryan::Solenoid>('A');
std::shared_ptr<ryan::Solenoid> puncherSolenoid = std::make_shared<ryan::Solenoid>('B');
std::shared_ptr<ryan::Solenoid> expansionSolenoid = std::make_shared<ryan::Solenoid>('G');

// SENSORS
std::shared_ptr<IMU> imu(new IMU(20));
std::shared_ptr<ADIButton> puncherLimitSwitch(new ADIButton('C'));

// MOTION PROFILE CONSTANTS
ryan::ProfileConstraint moveLimit({6_ftps, 10_ftps2, 10_ftps2, 34_ftps3}); //! todo!

// SUBSYSTEM CONTROLLERS
std::shared_ptr<ChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftChassis, rightChassis)
    .withDimensions(
        {AbstractMotor::gearset::blue, 1.0}, 
        {{2.75_in, 1.294_ft}, imev5BlueTPR}
    )
    .build();

std::shared_ptr<ryan::AsyncMotionProfiler> profiler = ryan::AsyncMotionProfilerBuilder()
    .withOutput(chassis)
    .withProfiler(std::make_unique<ryan::SCurveMotionProfile>(moveLimit))
    .build();

std::shared_ptr<IterativePosPIDController> turnPID = std::make_shared<IterativePosPIDController>
    (0.037, 0.0, 0.00065, 0, TimeUtilFactory::withSettledUtilParams(1, 2, 100_ms));

std::shared_ptr<Superstructure> superstructure = std::make_shared<Superstructure>
    (superstructureMotors, chassisSolenoid, puncherSolenoid, puncherLimitSwitch);