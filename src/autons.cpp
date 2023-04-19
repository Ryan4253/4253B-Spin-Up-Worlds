#include "autons.hpp"

void Autons::doNothing() {}

void Autons::jonathan() {
    auto path1 = squiggward->generate({{-1.95, 0.22, 0}, {-0.355, 1.64, 67 * degreeToRadian}});

    auto model = chassis->getModel();

    // superstructure->shoot();
    model->tank(-0.5, -0.5); pros::delay(150); model->tank(0, 0);
    superstructure->setIntake(12000); pros::delay(250); superstructure->setIntake(0);
    profiler->setTarget(6_in);
    pros::delay(500);
    superstructure->setIntake(12000);
    profiler->waitUntilSettled();

    pivotTurnToAngle(-45_deg, ChassisSide::LEFT);
    profiler->setTarget(-2_ft, true);

    profiler->setTarget(path1, {-1.95, 0.22, 0}, true, true);

    profiler->setTarget(Paths::Test, true, true);
}

void Autons::simple() {}

void Autons::skills() {}

void Autons::awp() {
    auto model = chassis->getModel();

    superstructure->shoot();
    model->tank(-0.5, -0.5); pros::delay(500); model->tank(0, 0);
    superstructure->setIntake(1); pros::delay(500); superstructure->setIntake(0);
}