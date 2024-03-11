#pragma once

#include <numbers>

#include <units/angular_velocity.h>
#include <units/velocity.h>

using units::meters_per_second_t;
using units::radians_per_second_t;

namespace constants {
    constexpr int k_NumberOfSwerveModules = 4;

    constexpr meters_per_second_t k_maxDriveSpeed = 3_mps;

    constexpr meters_per_second_t k_normalDriveSpeed = 1_mps;
    constexpr meters_per_second_t k_slowDriveSpeed = k_normalDriveSpeed * (1.0 - 0.20); // 20% slower.
    constexpr meters_per_second_t k_fastDriveSpeed = k_normalDriveSpeed * (1.0 + 0.25); // 25% faster.

    constexpr radians_per_second_t k_maxTurnSpeed = std::numbers::pi * 1_rad_per_s;

    constexpr radians_per_second_t k_slowTurnSpeed = k_maxTurnSpeed * (1.0 - 0.5);  // 50% slower.
};
