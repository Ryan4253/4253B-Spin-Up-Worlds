#pragma once
#include <iostream>
#include <vector>
#include "okapi/api/units/QTime.hpp"

namespace ryan{

/**
 * @brief Structure for trajectory points, which stores the robot's target position, velocity
 *        and acceleration at a specific time segment
 * 
 */
struct TrajectoryPoint {
    /**
     * @brief Construct an empty Trajectory Point 
     * 
     */
    TrajectoryPoint() = default;

    /**
     * @brief Destroy the Trajectory Point 
     * 
     */
    ~TrajectoryPoint() = default;
    
    /**
     * @brief Construct a new Trajectory Point object
     * 
     * @param leftP left position, in ft
     * @param rightP right position, in ft
     * @param leftV left velocity, in ft/s
     * @param rightV right velocity, in ft/s
     * @param leftA left acceleration, in ft/s^2
     * @param rightA right acceleration, in ft/s^2
     */
    TrajectoryPoint(double leftP, double rightP, double leftV, double rightV, double leftA, double rightA);

    /**
     * @brief forward declared in order to be able to access protected variables of this type
     */
    friend std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt);

    double leftPosition, rightPosition, leftVelocity, leftAcceleration, rightVelocity, rightAcceleration;
};

/**
 * @brief Prints the information of a trajectory point to an existing ostream
 * 
 * @param os the output stream to output to
 * @param pt the trajectory point to print
 * @return std::ostream& an ongoing output stream
 */
std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt);

/**
 * @brief class which stores a trajectory - a series of target position, velocity and acceleration
 *        which allows our robot to follow a predetermined path. The paths are pre-generated on 
 *        our computer and then deployed into our code.
 */
class Trajectory{
    private:
    std::vector<TrajectoryPoint> path;

    public:
    /**
     * @brief Construct a new Trajectory object
     * 
     */
    Trajectory() = default;
    
    /**
     * @brief Destroy the Trajectory object
     * 
     */
    ~Trajectory() = default;

    /**
     * @brief Construct a new Trajectory object
     * 
     * @param iPath the trajectory
     */
    Trajectory(const std::initializer_list<TrajectoryPoint>& iPath);
    
    /**
     * @brief gets the trajectory information at a given point of the trajectory
     * 
     * @param index the index (time) to check
     * @return TrajectoryPoint the trajectory point at the given time
     */
    TrajectoryPoint operator[](int index) const;

    /**
     * @brief returns the size of the trajectory. Every index stores 10ms of target kinematics
     * 
     * @return int the size of the trajectory
     */
    int size() const;
};

/**
 * @brief Struct for timed trajectory states (used with new Pathplanner)
 *
 */
struct TimedTrajectoryState {
    /**
     * @brief Construct a new Timed Trajectory State object
     *
     */
    TimedTrajectoryState() = default;

    /**
     * @brief Construct a new Timed Trajectory State object
     *
     * @param itime time stamp
     * @param ix x position
     * @param iy y position
     * @param itheta heading
     */
    TimedTrajectoryState(double itime, double ix, double iy, double itheta, double ilinVel, double iangularVel);

    double time, x, y, theta, linearVel, angularVel;
};

/**
 * @brief Class for timed trajectory (vector of timed trajectory states)
 *        (used with new Pathplanner)
 *
 */
class TimedTrajectory {
    private:
    std::vector<TimedTrajectoryState> trajectory;
    double totalTimeSeconds;

    public:
    /**
     * @brief Construct a new Timed Trajectory object
     *
     */
    TimedTrajectory() = default;

    /**
     * @brief Construct a new Timed Trajectory object
     *
     * @param itrajectory list of timed trajectory states
     */
    TimedTrajectory(const std::initializer_list<TimedTrajectoryState> &itrajectory);

    /**
     * @brief Returns the timed trajectory state based on given index
     *
     * @param index index of the trajectory
     * @return timed trajectory state at the index
     */
    TimedTrajectoryState operator[](int index) const;

    double getTimeSeconds();

    /**
     * @brief Returns size of the trajectory
     *
     * @return size of trajectory
     */
    int size() const;
};

}