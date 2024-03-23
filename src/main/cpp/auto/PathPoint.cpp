#include "auto/PathPoint.h"

PathPoint::PathPoint(units::meter_t x, units::meter_t y, frc::Rotation2d rotation, units::meters_per_second_t velocity) : x(x), y(y), rotation(rotation), velocity(velocity) {}

frc::Pose2d PathPoint::Pose() const {
    return frc::Pose2d{
        X(),
        Y(),
        Rotation()
    };
}

units::meter_t PathPoint::X() const { return x; }
units::meter_t PathPoint::Y() const { return y; }
frc::Rotation2d PathPoint::Rotation() const { return rotation; }
units::meters_per_second_t PathPoint::Velocity() const { return velocity; }