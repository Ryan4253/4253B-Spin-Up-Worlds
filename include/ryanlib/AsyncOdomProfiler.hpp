#pragma once
#include "TaskWrapper.hpp"
#include "Trajectory.hpp"
#include "StateMachine.hpp"
#include "FeedForward.hpp"
#include "LinearMotionProfile.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/api/filter/emaFilter.hpp"
#include "okapi/squiggles/squiggles.hpp"
#include "util/ramsete.hpp"
#include "util/pose.hpp"

namespace ryan{

/**
 * @brief an enum containing all possible states for our motion profile controller
 * 
 */
enum class OdomMotionProfileState{
    MOVE, FOLLOW, TURN, IDLE, SQUIGGLES_FOLLOW
};

// forward declare
template class StateMachine<OdomMotionProfileState>;

/**
 * @brief class that allows us to control our chassis asynchronously using motion profiles
 * 
 */
class AsyncOdomMotionProfiler : public StateMachine<OdomMotionProfileState, OdomMotionProfileState::IDLE>, public TaskWrapper {
    protected:
    /**
     * @brief Construct a new Async Motion Profiler object (using all custom velocity control)
     * 
     * @param iChassis chassis to output to
     * @param iMove linear motion profile constraint to generate
     * @param iLeftLinear left velocity controller for linear motion profile
     * @param iRightLinear right velocity controller for linear motion profile
     * @param iLeftTrajectory left velocity controller for trajectories
     * @param iRightTrajectory right velocity controller for trajectories
     * @param iTimeUtil timer utility
     */
    AsyncOdomMotionProfiler(std::shared_ptr<okapi::OdomChassisController> iChassis, 
                        std::unique_ptr<LinearMotionProfile> iMove, 
                        std::unique_ptr<FFVelocityController> iLeftLinear, 
                        std::unique_ptr<FFVelocityController> iRightLinear,
                        std::unique_ptr<FFVelocityController> iLeftTrajectory,
                        std::unique_ptr<FFVelocityController> iRightTrajectory,
                        std::unique_ptr<squiggles::SplineGenerator> iPathGen,
                        const okapi::TimeUtil& iTimeUtil);

    /**
     * @brief Construct a new Async Motion Profiler object (using internal velocity control)
     * 
     * @param iChassis chassis to output to
     * @param iMove linear motion profile constraint to generate
     * @param iTimeUtil timer utility
     */
    AsyncOdomMotionProfiler(std::shared_ptr<okapi::OdomChassisController> iChassis, 
                    std::unique_ptr<LinearMotionProfile> iMove, 
                    std::unique_ptr<squiggles::SplineGenerator> iPathGen,
                    const okapi::TimeUtil& iTimeUtil);

    /**
     * @brief Construct a new Async Motion Profiler object (using either custom or internal velocity control depending on the constructor)
     * 
     * @param iChassis chassis to output to
     * @param iMove linear motion profile to generate
     * @param iLeft left velocity controller (true for linear, false for trajectory)
     * @param iRight right velocity controller (true for linear, false for trajectory)
     * @param velFlag whether linear is custom (true) or trajectory is custom (true)
     * @param iTimeUtil timer utility
     */
    AsyncOdomMotionProfiler(std::shared_ptr<okapi::OdomChassisController> iChassis, 
                    std::unique_ptr<LinearMotionProfile> iMove, 
                    std::unique_ptr<FFVelocityController> iLeft,
                    std::unique_ptr<FFVelocityController> iRight,
                    bool velFlag,
                    std::unique_ptr<squiggles::SplineGenerator> iPathGen,
                    const okapi::TimeUtil& iTimeUtil);

    void operator=(const AsyncOdomMotionProfiler& rhs) = delete;

    /**
     * @brief generate this class using AsyncOdomMotionProfilerBuilder only
     * 
     */
    friend class AsyncOdomMotionProfilerBuilder;

    public:

    /**
     * @brief sets the motion profile constraints. Note that any movement currently running will be interrupted
     * 
     * @param iConstraint the kinematic constraints for the motion profile
     */
    void setConstraint(const ProfileConstraint& iConstraint);

    /**
     * @brief Set the target distance to move
     * 
     * @param iDistance target distance
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */
    void setTarget(okapi::QLength iDistance, bool waitUntilSettled = false);

    /**
     * @brief Set the target trajectory to follow
     * 
     * @param iPath target trajectory to follow
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */ 
    void setTarget(const Trajectory& iPath, bool waitUntilSettled = false);

    /**
     * @brief Set the target angle to turn
     * 
     * @param iPath target angle to turn
     * @param waitUntilSettled whether or not to wait until the motion profile is settled
     */ 
    void setTarget(okapi::QAngle iAngle, bool waitUntilSettled = false);

    void setTarget(std::vector<squiggles::Pose> iPathPoints, bool withRamsete = false, bool waitUntilSettled = false);

    /**
     * @brief stop the chassis from moving
     * 
     */
    void stop();

    /**
     * @brief blocks the current movement until the current movement is complete
     * 
     */
    void waitUntilSettled();

    protected:
    std::shared_ptr<okapi::OdomChassisController> chassis;
    std::shared_ptr<okapi::AbstractMotor> leftMotor;
    std::shared_ptr<okapi::AbstractMotor> rightMotor;

    std::unique_ptr<LinearMotionProfile> profiler;
    std::unique_ptr<FFVelocityController> leftLinear{nullptr};
    std::unique_ptr<FFVelocityController> rightLinear{nullptr};
    std::unique_ptr<FFVelocityController> leftTrajectory{nullptr};
    std::unique_ptr<FFVelocityController> rightTrajectory{nullptr};

    std::unique_ptr<lib4253::RamseteController> ramsete{nullptr};
    std::unique_ptr<squiggles::SplineGenerator> squiggward{nullptr};

    okapi::TimeUtil timeUtil;
    std::unique_ptr<okapi::AbstractRate> rate;
    std::unique_ptr<okapi::AbstractTimer> timer;
    okapi::QTime maxTime{0.0};

    Trajectory path;
    std::vector<squiggles::ProfilePoint> squigglesPath; 
    squiggles::Pose desiredSquigglesPose{0, 0, 0};

    pros::Mutex lock;

    bool trajectoryCustom = false;
    bool linearCustom = false;
    bool ramseteEnabled = false;

    /**
     * @brief task loop
     * 
     */
    void loop() override;
};


/**
 * @brief An AsyncMotionProfile builder class which allows more intuitive instantiation of the class
 * 
 */
class AsyncOdomMotionProfilerBuilder{
    public:
    /**
     * @brief Constructs a new Async Motion Profiler Builder object
     * 
     */
    AsyncOdomMotionProfilerBuilder();

    /**
     * @brief Destroys the Async Motion Profiler Builder object
     * 
     */
    ~AsyncOdomMotionProfilerBuilder() = default;

    /**
     * @brief sets the chassis object for the profiler to output to
     * 
     * @param iChassis the chassis object to output to
     * @return AsyncOdomMotionProfilerBuilder& an ongoing builder
     */
    AsyncOdomMotionProfilerBuilder& withOutput(std::shared_ptr<okapi::OdomChassisController> iChassis);

    /**
     * @brief sets the motion profile generator to use for linear movements
     * 
     * @param iProfiler the profile generator
     * @return AsyncOdomMotionProfilerBuilder& 
     */
    AsyncOdomMotionProfilerBuilder& withProfiler(std::unique_ptr<LinearMotionProfile> iProfiler);

    /**
     * @brief sets the motor controller to use for linear movements
     * 
     * @param iLeft 
     * @param iRight 
     * @return AsyncOdomMotionProfilerBuilder& 
     */
    AsyncOdomMotionProfilerBuilder& withLinearController(FFVelocityController iLeft, FFVelocityController iRight);

    /**
     * @brief sets the motor controller to use when following paths    
     * 
     * @param iLeft 
     * @param iRight 
     * @return AsyncOdomMotionProfilerBuilder& 
     */
    AsyncOdomMotionProfilerBuilder& withTrajectoryController(FFVelocityController iLeft, FFVelocityController iRight);

    AsyncOdomMotionProfilerBuilder& withPathGen(std::unique_ptr<squiggles::SplineGenerator> iPathGen);

    /**
     * @brief builds the async motion profiler object with the specified parameters. The thread is started automaically
     * 
     * @return std::shared_ptr<AsyncOdomMotionProfiler> the built async motion profiler
     */
    std::shared_ptr<AsyncOdomMotionProfiler> build();

    private:
    std::unique_ptr<LinearMotionProfile> profile;
    std::shared_ptr<okapi::OdomChassisController> chassis;
    FFVelocityController leftL;
    FFVelocityController rightL;
    FFVelocityController leftT;
    FFVelocityController rightT;

    bool linearInit = false;
    bool trajInit = false;
    bool driveInit = false;
    bool profileInit = false;
};
}