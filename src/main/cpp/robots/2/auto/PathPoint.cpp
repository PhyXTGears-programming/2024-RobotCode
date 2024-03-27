#include "robots/2/auto/PathPoint.h"

using namespace ::robot2;

robot2::PathPoint::PathPoint(
    units::meter_t x,
    units::meter_t y,
    frc::Rotation2d rotation,
    units::meters_per_second_t velocity
) :
    x(x),
    y(y),
    rotation(rotation),
    velocity(velocity)
{
    type = TYPE_OTHER;
}

PathPoint & robot2::PathPoint::SetCommand(frc2::CommandPtr && command) {
  this->command = std::move(command);

  return *this;
}

PathPoint & robot2::PathPoint::SetCommand(std::optional<frc2::CommandPtr> && command) {
  this->command = std::move(command);

  return *this;
}

PathPoint & robot2::PathPoint::SetType(const PathPointType type) {
    this->type = type;

    return *this;
}

frc::Pose2d robot2::PathPoint::Pose() const {
    return frc::Pose2d{
        X(),
        Y(),
        Rotation()
    };
}

units::meter_t robot2::PathPoint::X() const { return x; }
units::meter_t robot2::PathPoint::Y() const { return y; }
frc::Rotation2d robot2::PathPoint::Rotation() const { return rotation; }
units::meters_per_second_t robot2::PathPoint::Velocity() const { return velocity; }
PathPointType robot2::PathPoint::Type() const { return type; }

frc2::Command * robot2::PathPoint::Command() {
    if (command) {
        return command->get();
    } else {
        return nullptr;
    }
}

bool robot2::PathPoint::HasCommand() const {
    return command.has_value();
}
