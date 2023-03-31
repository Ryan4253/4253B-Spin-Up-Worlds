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
        std::shared_ptr<OdomChassisController> iChassis,
        double iB = 2.0,
        double iZeta = 0.7
    );

    std::pair<QSpeed, QSpeed> getTargetVelocity(QSpeed vel, QAngularSpeed angularVel, const Pose& error);

    private:
    std::shared_ptr<OdomChassisController> chassis;
    Pose tolerance;
    double b;
    double zeta;
};

}