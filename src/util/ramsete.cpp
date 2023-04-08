#include "util/ramsete.hpp"

namespace lib4253{

RamseteController::RamseteController(QLength iTrackWidth, double iB, double iZeta){
    b = iB;
    zeta = iZeta;
    trackWidth = iTrackWidth;
}

std::pair<QSpeed, QSpeed> RamseteController::getTargetVelocity(const Pose& currentPose, const Pose& poseRef, QSpeed vel, QAngularSpeed angularVel){
    Pose poseError = poseRef.relativeTo(currentPose);

    // Aliases for equation readability
    const double eX = poseError.X().convert(meter);
    const double eY = poseError.X().convert(meter);
    const double eTheta = poseError.Theta().convert(radian);
    const double vRef = vel.convert(mps);
    const double omegaRef = angularVel.convert(radps);

    // k = 2ζ√(ω_ref² + b v_ref²)
    double k = 2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * std::pow(vRef, 2));
    
    double alteredVelMps = vRef * std::cos(poseError.Theta().convert(radian)) + k * eX,
    double alteredAngularVelRadps = omegaRef + k * eTheta + b * vRef * Math::sinc(eTheta) * eY;

    QSpeed vl = (alteredVelMps - trackWidth.convert(meter) / 2 * alteredAngularVelRadps) * mps;
    QSpeed vr = (alteredVelMps + trackWidth.convert(meter) / 2 * alteredAngularVelRadps) * mps;

    return {vl, vr};
}

}