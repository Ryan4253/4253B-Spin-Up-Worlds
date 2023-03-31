#pragma once
#include "display/lvgl.h"
#include<cmath>
#include "pros/rtos.hpp"

/**
 * @brief Auton selector - inspired by the selector found in the ARMS library
 *        https://github.com/purduesigbots/ARMS
 * 
 */
namespace Selector {

extern int auton;
void init(int hue, int default_auton, const char** autons);

}