#include "ryanlib/AsyncOdomProfiler.hpp"

namespace ryan{
    
AsyncOdomMotionProfiler::AsyncOdomMotionProfiler(std::shared_ptr<okapi::OdomChassisController> iChassis, 
                                         std::unique_ptr<LinearMotionProfile> iMove, 
                                         std::unique_ptr<FFVelocityController> iLeftLinear, 
                                         std::unique_ptr<FFVelocityController> iRightLinear,
                                         std::unique_ptr<FFVelocityController> iLeftTrajectory,
                                         std::unique_ptr<FFVelocityController> iRightTrajectory,
                                         std::unique_ptr<squiggles::SplineGenerator> iPathGen,
                                         const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil)
{
    std::cout << "\tAsyncOdomMotionProfiler :: Constructor\n";
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    leftLinear =  std::move(iLeftLinear);
    rightLinear = std::move(iRightLinear);
    leftTrajectory = std::move(iLeftTrajectory);
    rightTrajectory = std::move(iRightTrajectory);
    squiggward = std::move(iPathGen);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());
    linearCustom = true;
    trajectoryCustom = true;

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());

    std::unique_ptr<lib4253::RamseteController> ramsete(new lib4253::RamseteController(chassis));

    // squiggles::Constraints constraints = squiggles::Constraints(
    //     profiler->getConstraint().maxVelocity.convert(okapi::mps), 
    //     profiler->getConstraint().maxAcceleration.convert(okapi::mps2), 
    //     profiler->getConstraint().maxJerk.convert(okapi::mps3)
    // );
    // std::unique_ptr<squiggles::SplineGenerator> squiggward(new squiggles::SplineGenerator(
    //     constraints, 
    //     std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(okapi::meter), constraints), 
    //     0.01
    // ));
}

AsyncOdomMotionProfiler::AsyncOdomMotionProfiler(std::shared_ptr<okapi::OdomChassisController> iChassis, 
    std::unique_ptr<LinearMotionProfile> iMove, 
    std::unique_ptr<squiggles::SplineGenerator> iPathGen,
    const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil)
{
    std::cout << "    AsyncOdomMotionProfiler :: Constructor :)\n";
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    squiggward = std::move(iPathGen);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());

    std::unique_ptr<lib4253::RamseteController> ramsete(new lib4253::RamseteController(chassis));

    // squiggles::Constraints constraints = squiggles::Constraints(
    //     profiler->getConstraint().maxVelocity.convert(okapi::mps), 
    //     profiler->getConstraint().maxAcceleration.convert(okapi::mps2), 
    //     profiler->getConstraint().maxJerk.convert(okapi::mps3)
    // );
    // std::unique_ptr<squiggles::SplineGenerator> squiggward(new squiggles::SplineGenerator(
    //     constraints, 
    //     std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(okapi::meter), constraints), 
    //     0.01
    // ));
}

AsyncOdomMotionProfiler::AsyncOdomMotionProfiler(std::shared_ptr<okapi::OdomChassisController> iChassis, 
                std::unique_ptr<LinearMotionProfile> iMove, 
                std::unique_ptr<FFVelocityController> iLeft,
                std::unique_ptr<FFVelocityController> iRight,
                bool velFlag,
                std::unique_ptr<squiggles::SplineGenerator> iPathGen,
                const okapi::TimeUtil& iTimeUtil): timeUtil(iTimeUtil)
{
    std::cout << "AsyncOdomMotionProfiler :: Constructor\n";
    chassis = std::move(iChassis);
    profiler = std::move(iMove);
    if(velFlag){
        leftLinear = std::move(iLeft);
        rightLinear = std::move(iRight);
        linearCustom = true;
    }
    else{
        leftTrajectory = std::move(iLeft);
        rightTrajectory = std::move(iRight);
        trajectoryCustom = true;
    }
    squiggward = std::move(iPathGen);
    rate = std::move(timeUtil.getRate());
    timer = std::move(timeUtil.getTimer());

    leftMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getLeftSideMotor());
    rightMotor = std::move(std::static_pointer_cast<okapi::SkidSteerModel>(chassis->getModel())->getRightSideMotor());

    std::unique_ptr<lib4253::RamseteController> ramsete(new lib4253::RamseteController(chassis));

    // squiggles::Constraints constraints = squiggles::Constraints(
    //     profiler->getConstraint().maxVelocity.convert(okapi::mps), 
    //     profiler->getConstraint().maxAcceleration.convert(okapi::mps2), 
    //     profiler->getConstraint().maxJerk.convert(okapi::mps3)
    // );
    // std::unique_ptr<squiggles::SplineGenerator> squiggward(new squiggles::SplineGenerator(
    //     constraints, 
    //     std::make_shared<squiggles::TankModel>(chassis->getChassisScales().wheelTrack.convert(okapi::meter), constraints), 
    //     0.01
    // ));
}

void AsyncOdomMotionProfiler::setConstraint(const ProfileConstraint& iConstraint){
    stop();
    lock.take(5);
    profiler->setConstraint(iConstraint);
    lock.give();
}

void AsyncOdomMotionProfiler::setTarget(okapi::QLength distance, bool waitUntilSettled){
    std::cout << "AsyncOdomMotionProfiler :: Setting Target of " << distance.convert(okapi::foot) << " ft\n";
    lock.take(5);
    setState(OdomMotionProfileState::MOVE);
    profiler->setDistance(distance);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    maxTime = profiler->getTotalTime() + 0.02 * okapi::second;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
    std::cout << "done\n";
}

void AsyncOdomMotionProfiler::setTarget(const Trajectory& iPath, bool waitUntilSettled){
    std::cout << "AsyncOdomMotionProfiler :: Setting Target of Trajectory\n";
    lock.take(5);
    setState(OdomMotionProfileState::FOLLOW);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    path = iPath;
    maxTime = path.size() * 10 * okapi::millisecond + 0.02 * okapi::millisecond;
    timer->placeMark();
    lock.give();

    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncOdomMotionProfiler::setTarget(okapi::QAngle iAngle, bool waitUntilSettled){
    std::cout << "AsyncOdomMotionProfiler :: Setting Target of " << iAngle.convert(okapi::degree) << " deg\n";
    lock.take(5);
    setState(OdomMotionProfileState::TURN);
    profiler->setDistance(iAngle.convert(okapi::radian) * chassis->getChassisScales().wheelTrack/2);
    leftMotor->tarePosition();
    rightMotor->tarePosition();
    chassis->getModel()->tank(0, 0);
    maxTime = profiler->getTotalTime() + 0.02 * okapi::second;
    timer->placeMark();
    lock.give();
    
    if(waitUntilSettled){
        this->waitUntilSettled();
    }
}

void AsyncOdomMotionProfiler::setTarget(std::vector<squiggles::Pose> iPathPoints, bool withRamsete, bool waitUntilSettled) {
    std::cout << "AsyncOdomMotionProfiler :: Setting Target of Squiggles Path\n";

    lock.take(5);
    std::cout << "a\n";
    setState(OdomMotionProfileState::SQUIGGLES_FOLLOW);
    std::cout << "b\n";
    leftMotor->tarePosition();
    std::cout << "c\n";
    rightMotor->tarePosition();
    std::cout << "d\n";
    chassis->getModel()->tank(0, 0);
    std::cout << "e\n";
    squigglesPath = squiggward->generate({{1, 0, 0}, {0, 1, 0}});
    std::cout << "out: " << squigglesPath[10].wheel_velocities[1];
    // squiggward->generate(iPathPoints);
    // squigglesPath = squiggward->generate(iPathPoints);
    std::cout << "g\n";
    desiredSquigglesPose = iPathPoints[0];
    std::cout << "h\n";
    ramseteEnabled = withRamsete;
    std::cout << "i\n";
    maxTime = squigglesPath.size() * 10 * okapi::millisecond + 0.02 * okapi::second;
    std::cout << "j\n";
    timer->placeMark();
    std::cout << "k\n";
    lock.give();

    std::cout<<"target setted\n";
    if(waitUntilSettled){
        this->waitUntilSettled();
    }
    
}

void AsyncOdomMotionProfiler::stop(){
    std::cout << "AsyncOdomMotionProfiler :: Stopping\n";
    lock.take(5);
    setState(OdomMotionProfileState::IDLE);
    (chassis->getModel())->tank(0, 0);
    lock.give();
}

void AsyncOdomMotionProfiler::loop(){
    TrajectoryPoint pt;
    double leftPower, rightPower;
    okapi::EmaFilter lFilter(0.7);
    okapi::EmaFilter rFilter(0.7);

    while(true){
        lock.take(5);
        okapi::QTime time = timer->getDtFromMark();

        double leftPos = Math::tickToFt(leftMotor->getPosition(), chassis->getChassisScales(), chassis->getGearsetRatioPair());
        double leftVel = Math::rpmToFtps(leftMotor->getActualVelocity(), chassis->getChassisScales(), chassis->getGearsetRatioPair());
        double rightPos = Math::tickToFt(rightMotor->getPosition(), chassis->getChassisScales(), chassis->getGearsetRatioPair());
        double rightVel = Math::rpmToFtps(rightMotor->getActualVelocity(), chassis->getChassisScales(), chassis->getGearsetRatioPair());

        // std::cout << lFilter.filter(leftVel) << std::endl;
        //std::cout << rFilter.filter(rightVel) << std::endl;
        //std::cout << leftPos << std::endl;
        //std::cout << rightPos << std::endl;

        if(getState() == OdomMotionProfileState::IDLE){

        }
        else if(time > maxTime){
            setState(OdomMotionProfileState::IDLE);
            chassis->getModel()->tank(0, 0);
        }
        else if(getState() == OdomMotionProfileState::MOVE){
            pt = profiler->get(time);
            if(linearCustom){
                leftPower = leftLinear->step(pt.leftPosition, pt.leftVelocity, pt.leftAcceleration, leftPos, leftVel);
                rightPower = rightLinear->step(pt.rightPosition, pt.rightVelocity, pt.rightAcceleration, rightPos, rightVel);
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double vel = Math::ftpsToRPM(pt.leftVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(vel);
                rightMotor->moveVelocity(vel);
            }
        }
        else if(getState() == OdomMotionProfileState::TURN){
            pt = profiler->get(time);
            if(linearCustom){
                leftPower = leftLinear->step(pt.leftPosition, pt.leftVelocity, pt.leftAcceleration, leftPos, leftVel);
                rightPower = rightLinear->step(-pt.rightPosition, -pt.rightVelocity, -pt.rightAcceleration, rightPos, rightVel);
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double vel = Math::ftpsToRPM(pt.leftVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(vel);
                rightMotor->moveVelocity(-vel);
            }
        }
        else if(getState() == OdomMotionProfileState::FOLLOW){
            pt = path[(int)(time.convert(okapi::millisecond) / 10)];
            if(trajectoryCustom){
                leftPower = leftTrajectory->step(pt.leftPosition, pt.leftVelocity, pt.leftAcceleration, leftPos, leftVel);
                rightPower = rightTrajectory->step(pt.rightPosition, pt.rightVelocity, pt.rightAcceleration, rightPos, rightVel);
                chassis->getModel()->tank(leftPower, rightPower);
            }
            else{
                double leftVel = Math::ftpsToRPM(pt.leftVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                double rightVel = Math::ftpsToRPM(pt.rightVelocity, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(leftVel);
                rightMotor->moveVelocity(rightVel);
            }
        }
        else if(getState() == OdomMotionProfileState::SQUIGGLES_FOLLOW) {
            std::cout << "hello\n";
            squiggles::ProfilePoint point = squigglesPath[(int)(time.convert(okapi::millisecond) / 10)];
            std::cout << "a\n";
            double leftVel = point.wheel_velocities[0];
            std::cout << "b\n";
            double rightVel = point.wheel_velocities[1];
            std::cout << "hi\n";
            if(ramseteEnabled) {
                std::cout << "ramsete\n";
                double desiredVelMPS = (leftVel + rightVel) / 2;
                double desiredAngularVelRPS = (rightVel - leftVel) / chassis->getChassisScales().wheelTrack.convert(okapi::meter);
                double deltaDist = desiredVelMPS * 0.01;
                double deltaAngle = desiredAngularVelRPS * 0.01;
                squiggles::Pose deltaPose = {(std::sin(deltaAngle) * deltaDist), (std::sin(deltaAngle) * deltaDist), deltaAngle};
                desiredSquigglesPose = {desiredSquigglesPose.x + deltaPose.x, desiredSquigglesPose.y + deltaPose.y, desiredSquigglesPose.yaw + deltaPose.yaw};
                squiggles::Pose currPose = {chassis->getState().x.convert(okapi::meter), chassis->getState().y.convert(okapi::meter), chassis->getState().theta.convert(okapi::radian)};
                squiggles::Pose poseError = {desiredSquigglesPose.x - currPose.x, desiredSquigglesPose.y - currPose.y, desiredSquigglesPose.yaw - currPose.yaw};

                std::pair<okapi::QSpeed, okapi::QSpeed> adjustedVel = ramsete->getTargetVelocity(
                    desiredVelMPS * okapi::mps, 
                    desiredAngularVelRPS * okapi::radps, 
                    {poseError.x * okapi::meter, poseError.y * okapi::meter, poseError.yaw * okapi::radian}
                );
                double leftRPM = Math::ftpsToRPM(adjustedVel.first.convert(okapi::ftps), chassis->getChassisScales(), chassis->getGearsetRatioPair());
                double rightRPM = Math::ftpsToRPM(adjustedVel.second.convert(okapi::ftps), chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(leftRPM);
                rightMotor->moveVelocity(rightRPM);
            } else {
                std::cout << "no ramsete\n";
                double leftRPM = Math::ftpsToRPM(leftVel, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                double rightRPM = Math::ftpsToRPM(rightVel, chassis->getChassisScales(), chassis->getGearsetRatioPair());
                leftMotor->moveVelocity(leftRPM);
                rightMotor->moveVelocity(rightRPM);
                std::cout<<"left RPM :: " << leftRPM << "    right RPM :: " << rightRPM << std::endl;
            }
        }

        lock.give();
        rate->delayUntil(10);
    }
}

void AsyncOdomMotionProfiler::waitUntilSettled(){
    while(getState() != OdomMotionProfileState::IDLE){
        pros::delay(10);
    }
}

AsyncOdomMotionProfilerBuilder::AsyncOdomMotionProfilerBuilder(){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Building...\n\n";
    linearInit = false;
    trajInit = false;
    driveInit = false;
    profileInit = false;
}

AsyncOdomMotionProfilerBuilder& AsyncOdomMotionProfilerBuilder::withOutput(std::shared_ptr<okapi::OdomChassisController> iChassis){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Adding Output\n";
    chassis = std::move(iChassis);
    driveInit = true;
    return *this;
}

AsyncOdomMotionProfilerBuilder& AsyncOdomMotionProfilerBuilder::withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Adding Profiler\n";
    profile = std::move(iProfiler);
    profileInit = true;
    return *this;
}

AsyncOdomMotionProfilerBuilder& AsyncOdomMotionProfilerBuilder::withLinearController(FFVelocityController iLeft, FFVelocityController iRight){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Adding Linear Controller\n";
    leftL = iLeft, rightL = iRight;
    linearInit = true;
    return *this;

}

AsyncOdomMotionProfilerBuilder& AsyncOdomMotionProfilerBuilder::withTrajectoryController(FFVelocityController iLeft, FFVelocityController iRight){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Adding Trajectory Controller\n";
    leftT = iLeft, rightT = iRight;
    trajInit = true;
    return *this;
}

AsyncOdomMotionProfilerBuilder& AsyncOdomMotionProfilerBuilder::withPathGen(std::unique_ptr<squiggles::SplineGenerator> iPathGen){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Adding Path Gen\n";
    pathGen = std::move(iPathGen);
    pathGenInit = true;
    return *this;
}

std::shared_ptr<AsyncOdomMotionProfiler> AsyncOdomMotionProfilerBuilder::build(){
    std::cout << "AsyncOdomMotionProfilerBuilder :: Finalizing Build...\n";
    if(driveInit && profileInit && linearInit && trajInit && pathGenInit){
        std::shared_ptr<AsyncOdomMotionProfiler> ret(new AsyncOdomMotionProfiler(std::move(chassis), 
                                            std::move(profile), 
                                            std::make_unique<FFVelocityController>(leftL), 
                                            std::make_unique<FFVelocityController>(rightL),
                                            std::make_unique<FFVelocityController>(leftT), 
                                            std::make_unique<FFVelocityController>(rightT),
                                            std::move(pathGen),
                                            okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        std::cout << "AsyncOdomMotionProfilerBuilder :: Build Successful!\n\n";
        return std::move(ret);
    }
    else if(driveInit && profileInit && linearInit && pathGenInit){
        std::shared_ptr<AsyncOdomMotionProfiler> ret(new AsyncOdomMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::make_unique<FFVelocityController>(leftL), 
                                    std::make_unique<FFVelocityController>(rightL),
                                    true,
                                    std::move(pathGen),
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        std::cout << "AsyncOdomMotionProfilerBuilder :: Build Successful!\n\n";
        return std::move(ret);
    }
    else if(driveInit && profileInit && trajInit && pathGenInit){
        std::shared_ptr<AsyncOdomMotionProfiler> ret(new AsyncOdomMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::make_unique<FFVelocityController>(leftT), 
                                    std::make_unique<FFVelocityController>(rightT),
                                    false,
                                    std::move(pathGen),
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        std::cout << "AsyncOdomMotionProfilerBuilder :: Build Successful!\n\n";
        return std::move(ret);
    }
    else if(driveInit && profileInit && pathGenInit){
        std::shared_ptr<AsyncOdomMotionProfiler> ret(new AsyncOdomMotionProfiler(std::move(chassis), 
                                    std::move(profile), 
                                    std::move(pathGen),
                                    okapi::TimeUtilFactory::createDefault()));

        ret->startTask();
        std::cout << "AsyncOdomMotionProfilerBuilder :: Build Successful!\n\n";
        return std::move(ret);
    }
    else{
        throw std::runtime_error("AsyncOdomMotionProfilerBuilder: Not all parameters supplied, failed to build (you need at least a chassis and a profiler");
    }
}

}