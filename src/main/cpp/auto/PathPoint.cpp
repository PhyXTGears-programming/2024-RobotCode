#include "auto/PathPoint.h"

PathPoint::PathPoint(units::meter_t x, units::meter_t y, frc::Rotation2d rotation, units::meters_per_second_t velocity) : x(x), y(y), rotation(rotation), velocity(velocity) {
    type = TYPE_OTHER;
}

PathPoint & PathPoint::SetCommand(frc2::CommandPtr && command) {
  this->command = std::move(command);

  return *this;
}

PathPoint & PathPoint::SetCommand(std::optional<frc2::CommandPtr> && command) {
  this->command = std::move(command);

  return *this;
}

PathPoint & PathPoint::SetType(const PathPointType type) {
    this->type = type;

    return *this;
}

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
PathPointType PathPoint::Type() const { return type; }

frc2::Command * PathPoint::Command() {
    if (command) {
        return command->get();
    } else {
        return nullptr;
    }
}

bool PathPoint::HasCommand() const {
    return command.has_value();
}
