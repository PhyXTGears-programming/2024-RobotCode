#pragma once

#include <numbers>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

using units::meters_per_second_t;
using units::radians_per_second_t;
using units::revolutions_per_minute_t;

using rev_per_min_per_volt = units::compound_unit<units::turn, units::inverse<units::minute>, units::inverse<units::volt>>;
using rev_per_min_per_volt_t = units::unit_t<rev_per_min_per_volt>;

using rad_per_sec_per_volt = units::compound_unit<units::radian, units::inverse<units::second>, units::inverse<units::volt>>;
using rad_per_sec_per_volt_t = units::unit_t<rad_per_sec_per_volt>;

namespace constants {

    constexpr meters_per_second_t k_maxDriveSpeed = 3_mps;

    constexpr meters_per_second_t k_normalDriveSpeed = 1_mps;
    constexpr meters_per_second_t k_slowDriveSpeed = k_normalDriveSpeed * (1.0 - 0.20); // 20% slower.
    constexpr meters_per_second_t k_fastDriveSpeed = k_normalDriveSpeed * (1.0 + 0.25); // 25% faster.

    constexpr radians_per_second_t k_maxTurnSpeed = std::numbers::pi * 1_rad_per_s;

    constexpr radians_per_second_t k_slowTurnSpeed = k_maxTurnSpeed * (1.0 - 0.5);  // 50% slower.

    namespace drive {
        constexpr int k_numberOfSwerveModules = 4;

        constexpr units::meter_t k_wheelDiameter = 3.75_in;

        constexpr double k_turnWheelPerMotorRatio  = 21.428571;
        constexpr double k_driveWheelPerMotorRatio =  0.104056;

        constexpr rev_per_min_per_volt_t k_driveRpmPerVolt(78.061404);
        constexpr rev_per_min_per_volt_t k_driveRadPerSecPerVolt = k_driveRpmPerVolt.convert<rad_per_sec_per_volt>();
    }
};
