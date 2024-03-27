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

namespace robot1::constants {

    constexpr meters_per_second_t k_maxDriveSpeed = 4_mps;

    constexpr meters_per_second_t k_normalDriveSpeed = 1.5_mps;
    constexpr meters_per_second_t k_slowDriveSpeed = k_normalDriveSpeed * (1.0 - 0.75); // 75% slower.
    constexpr meters_per_second_t k_fastDriveSpeed = 4_mps;

    constexpr radians_per_second_t k_maxTurnSpeed = 360_deg_per_s;

    constexpr radians_per_second_t k_normalTurnSpeed = 180_deg_per_s;
    constexpr radians_per_second_t k_slowTurnSpeed = k_normalTurnSpeed * (1.0 - 0.5);  // 50% slower.
    constexpr radians_per_second_t k_fastTurnSpeed = k_maxTurnSpeed;

    namespace autonomous {
        const std::string k_None = "None";
        const std::string k_ShootSpeakerAndStay = "Shoot Speaker & Stay";
        const std::string k_ShootSpeakerAndLeave = "Shoot Speaker & Leave";
        const std::string k_ShootTwo = "Shoot Two";
        const std::string k_FollowPath = "Follow Path";
        const std::string k_Subwoof2n = "Subwoofer 2n";
        const std::string k_Subwoof3n = "Subwoofer 3n";
    }

    namespace drive {
        constexpr int k_numberOfSwerveModules = 4;

        constexpr units::meter_t k_wheelDiameter = 3.75_in;

        constexpr double k_turnWheelPerMotorRatio  = 21.428571;
        constexpr double k_driveWheelPerMotorRatio = 0.164062;

        constexpr rev_per_min_per_volt_t k_driveRpmPerVolt(78.061404);
        constexpr rev_per_min_per_volt_t k_driveRadPerSecPerVolt = k_driveRpmPerVolt.convert<rad_per_sec_per_volt>();
    }
};
