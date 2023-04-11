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
std::shared_ptr<ryan::Solenoid> chassisSolenoid(new ryan::Solenoid({{14, 'F'}}));
std::shared_ptr<ryan::Solenoid> puncherSolenoid(new ryan::Solenoid({{14, 'G'}}));
std::shared_ptr<ryan::Solenoid> expansionSolenoid(new ryan::Solenoid({{14, 'E'}}));
std::shared_ptr<ryan::Solenoid> intakeSolenoid(new ryan::Solenoid('A'));

// SENSORS
std::shared_ptr<IMU> imu(new IMU(17));
std::shared_ptr<ADIButton> puncherLimitSwitch(new ADIButton(std::make_pair<std::uint8_t, std::uint8_t>(14, 'H')));
// std::shared_ptr<ADIEncoder> middleTracker(new ADIEncoder('A', 'B'));

// MOTION PROFILE CONSTANTS
ryan::ProfileConstraint moveLimit({5.25_ftps, 10_ftps2, 10_ftps2, 34_ftps3}); //! todo!

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
QSpeed profileMaxVel = 5.25_ftps;
QAcceleration profileMaxAccel = 10_ftps2;
QJerk profileMaxJerk = 34_ftps3;
squiggles::Constraints constraints(profileMaxVel.convert(mps), profileMaxAccel.convert(mps2), profileMaxJerk.convert(mps3));

std::shared_ptr<squiggles::SplineGenerator> squiggward(new squiggles::SplineGenerator(
    constraints, 
    std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(okapi::meter), constraints), 
    0.01
));

std::shared_ptr<ryan::AsyncOdomMotionProfiler> profiler = ryan::AsyncOdomMotionProfilerBuilder()
    .withOutput(chassis)
    .withProfiler(std::make_unique<ryan::SCurveMotionProfile>(moveLimit))
    .build();

std::shared_ptr<IterativePosPIDController> turnPID(new IterativePosPIDController
    (0.024, 0.0001, 0.00067, 0, TimeUtilFactory::withSettledUtilParams(1.5, 2, 100_ms)));

std::shared_ptr<Superstructure> superstructure(new Superstructure(
    leftSuperstructure, rightSuperstructure, 
    chassisSolenoid, puncherSolenoid, 
    puncherLimitSwitch
));