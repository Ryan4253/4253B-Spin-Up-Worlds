#include "autons.hpp"

void Autons::doNothing() {}

void Autons::jonathan() {}

void Autons::simple() {}

void Autons::skills() {}

void Autons::awp() {
    auto model = chassis->getModel();

    superstructure->shoot();
    model->tank(-0.5, -0.5); pros::delay(500); model->tank(0, 0);
    superstructure->setIntake(1); pros::delay(500); superstructure->setIntake(0);
}