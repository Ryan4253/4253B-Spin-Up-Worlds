#include "Trajectory.hpp"

namespace ryan{

TrajectoryPoint::TrajectoryPoint(double leftP, double rightP, double leftV, double rightV, double leftA, double rightA): 
leftPosition(leftP), rightPosition(rightP), leftVelocity(leftV), leftAcceleration(leftA), rightVelocity(rightV), rightAcceleration(rightA){}

std::ostream& operator<<(std::ostream& os, TrajectoryPoint& pt){
    os << pt.leftPosition << " " << pt.rightPosition << " " << pt.leftVelocity << " " << pt.rightVelocity << " " << pt.leftAcceleration << " " << pt.rightAcceleration;
    return os;
}

Trajectory::Trajectory(const std::initializer_list<TrajectoryPoint>& iPath):
path(iPath)
{}

TrajectoryPoint Trajectory::operator[](int index) const{
    if(index < 0 || index >= path.size()){
        return {0, 0, 0, 0, 0, 0};
    }
    else{
        return path[index];
    }
}

int Trajectory::size() const{
    return path.size();
}

/** Timed Trajectory State */
TimedTrajectoryState::TimedTrajectoryState(double itime, double ix, double iy, double itheta, double ilinVel, double iangularVel)
    : time(itime), x(ix), y(iy), theta(itheta), linearVel(ilinVel), angularVel(iangularVel) {}

/** Timed Trajectory */
TimedTrajectory::TimedTrajectory(const std::initializer_list<TimedTrajectoryState> &itrajectory) {
    trajectory = itrajectory;
    totalTimeSeconds = trajectory[trajectory.size()-1].time;
}

int TimedTrajectory::size() const {
    return trajectory.size();
}

double TimedTrajectory::getTimeSeconds() {
    return totalTimeSeconds;
}

TimedTrajectoryState TimedTrajectory::operator[](int index) const {
    if (index < 0 || index >= trajectory.size()) {
        return {0, 0, 0, 0, 0, 0};
    }
    return trajectory[index];
}

}