#include "globals.hpp"

// CONTROLLERS
std::shared_ptr<Controller> master(new Controller(okapi::ControllerId::master));

// MOTORS
Motor leftFront(2, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftMid(3, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(4, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightFront(7, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightMid(8, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(9, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<Motor> leftSuperstructure(new Motor(5, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees));
std::shared_ptr<Motor> rightSuperstructure(new Motor(6, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees));

std::shared_ptr<MotorGroup> leftChassis(new MotorGroup({leftFront, leftMid, leftBack}));
std::shared_ptr<MotorGroup> rightChassis(new MotorGroup({rightFront, rightMid, rightBack}));

// SOLENOIDS
std::shared_ptr<ryan::Solenoid> chassisSolenoid(new ryan::Solenoid('D'));
std::shared_ptr<ryan::Solenoid> puncherSolenoid(new ryan::Solenoid('B'));
std::shared_ptr<ryan::Solenoid> expansionSolenoid(new ryan::Solenoid('C'));
std::shared_ptr<ryan::Solenoid> intakeSolenoid(new ryan::Solenoid('A'));
std::shared_ptr<ryan::Solenoid> bandReleaseSolenoid(new ryan::Solenoid('E'));

// SENSORS
std::shared_ptr<IMU> imu(new IMU(14));
std::shared_ptr<RotationSensor> puncherEncoder(new RotationSensor(16, true));

// MOTION PROFILE CONSTANTS
ryan::ProfileConstraint moveLimit({5_ftps, 8_ftps2, 8_ftps2, 34_ftps3}); //! todo!

// SUBSYSTEM CONTROLLERS
std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftChassis, rightChassis)
    .withDimensions(
        {AbstractMotor::gearset::blue, 1.5}, 
        {{3.25_in, 28_cm}, 300 * 1.5}
    )
    .withOdometry()
    .buildOdometry();

QSpeed theoreticalMaxSpeed = 5.672_ftps;
QSpeed profileMaxVel = 4.5_ftps;
QAcceleration profileMaxAccel = 5_ftps2;
QJerk profileMaxJerk = 34_ftps3;
QAngularSpeed profileMaxCurvature = 3 * okapi::radps;
squiggles::Constraints constraints(
    profileMaxVel.convert(ftps), 
    profileMaxAccel.convert(ftps2), 
    profileMaxJerk.convert(ftps3)
    // profileMaxCurvature.convert(radps)
);

std::shared_ptr<squiggles::SplineGenerator> squiggward(new squiggles::SplineGenerator(
    constraints, 
    std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(okapi::foot), constraints), 
    0.01
));

std::shared_ptr<ryan::AsyncOdomMotionProfiler> profiler = ryan::AsyncOdomMotionProfilerBuilder()
    .withOutput(chassis)
    .withProfiler(std::make_unique<ryan::SCurveMotionProfile>(moveLimit))
    .build();

std::shared_ptr<IterativePosPIDController> pointTurnPID(new IterativePosPIDController
    (0.025, 0.0001, 0.0005, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms)));

std::shared_ptr<IterativePosPIDController> pivotTurnPID(new IterativePosPIDController
    (0.045, 0.0, 0.0, 0.0001, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms)));

std::shared_ptr<Superstructure> superstructure(new Superstructure(
    leftSuperstructure, rightSuperstructure, 
    chassisSolenoid, puncherSolenoid, 
    puncherEncoder
));