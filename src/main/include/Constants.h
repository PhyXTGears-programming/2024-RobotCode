#pragma once 
#include "Interface.h"
#include <cmath>
#include <numbers>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace Constants {
    const int k_NumberOfSwerveModules = 4;

    constexpr double k_minDriveSpeed = 0.20;
    constexpr double k_normalDriveSpeed = 0.50;
    constexpr double k_maxDriveSpeed = 1.00;
    constexpr double k_maxSpinSpeed = std::numbers::pi;

    const double k_kickstandServoleReleaseAngle = 120;
};
