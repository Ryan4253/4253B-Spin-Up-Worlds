#pragma once

#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"

#include "util/math.hpp"
#include "util/pose.hpp"

#include <memory>

namespace lib4253{

using namespace okapi;

class RamseteController{
    public:
    RamseteController(
        QLength iTrackWidth,
        double iB = 2.0,
        double iZeta = 0.7
    );

    std::pair<QSpeed, QSpeed> getTargetVelocity(const Pose& currentPose, const Pose& poseRef, QSpeed vel, QAngularSpeed angularVel);

    private:
    Pose tolerance;
    double b;
    double zeta;
    QLength trackWidth;
};

}