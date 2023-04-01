#include "util/ramsete.hpp"

namespace lib4253{

RamseteController::RamseteController(std::shared_ptr<OdomChassisController> iChassis, double iB, double iZeta){
    b = iB;
    zeta = iZeta;
    chassis = std::move(iChassis);
    trackWidth = chassis->getChassisScales().wheelTrack;
}

std::pair<QSpeed, QSpeed> RamseteController::getTargetVelocity(QSpeed vel, QAngularSpeed angularVel, const Pose& error){
    // Aliases for equation readability
    double eX = error.X().convert(okapi::meter);
    double eY = error.Y().convert(okapi::meter);
    double eTheta = error.Theta().convert(okapi::radian);
    double vRef = vel.convert(okapi::mps);
    double omegaRef = angularVel.convert(okapi::radps);

    // Unit inconsistent dark magic ?!?!??!?!!!??!?!
    double k = 2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * vRef * vRef);

    QSpeed v{(vRef * cos(eTheta) + k * eX) * okapi::mps};
    QAngularSpeed omega{(omegaRef + k * eTheta + b * vRef * Math::sinc(eTheta) * eY) * okapi::radps};

    QSpeed vl = (2 * vl + ((omega.convert(radps) * chassis->getChassisScales().wheelTrack.convert(meter)) * mps)) / 2;
    QSpeed vr = (2 * vl - ((omega.convert(radps) * chassis->getChassisScales().wheelTrack.convert(meter)) * mps)) / 2;

    return {vl, vr};
}

}