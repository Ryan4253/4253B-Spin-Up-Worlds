#include "autons.hpp"

void Autons::doNothing() {}

void Autons::rollerRight(){
    profiler->setTarget(-0.7_tile, true);
    pointTurnToAngle(90_deg);
    superstructure->setIntake(12000);
    profiler->setTarget(-0.3_tile);
    pros::delay(800);
    superstructure->setIntake(0);
    profiler->waitUntilSettled();

}

void Autons::sixDiscRight(){
    superstructure->setIntake(12000);
    profiler->setTarget(1.4_tile, true);
    pointTurnToAngle(30_deg);
    superstructure->shoot();
}

void Autons::nineDiscRight(){

}

void Autons::sixDiscAWP() {
    auto model = chassis->getModel();

    superstructure->shoot();
    model->tank(-0.5, -0.5); pros::delay(500); model->tank(0, 0);
    superstructure->setIntake(1); pros::delay(500); superstructure->setIntake(0);
}