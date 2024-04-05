#pragma once

#include "robots/1/auto/load/SubsystemRegistry.h"
#include "common/auto/PathPoint.h"
#include "robots/1/subsystems/drivetrain/Drivetrain.h"
#include "util/geom.h"
#include "util/vector.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>

#include <units/velocity.h>

#include <wpi/json.h>
#include <wpi/MemoryBuffer.h>

namespace robot1 {

    std::vector<frc::Pose2d> loadPosePathFromJSON(wpi::json &path);
    std::vector<PathPoint> loadPathFromJSON(wpi::json &path, SubsystemRegistry & registry);

    frc2::CommandPtr loadPoseFollowCommandFromFile(Drivetrain *m_drivetrain, std::string_view filename);
    frc2::CommandPtr loadPathFollowCommandFromFile(std::string_view filename, SubsystemRegistry & registry);

    frc2::CommandPtr generatePathFollowCommand(std::vector<frc::Pose2d> path, units::meters_per_second_t speed, Drivetrain *c_drivetrain);
    frc2::CommandPtr generatePathFollowCommand(std::vector<PathPoint> && path, Drivetrain *c_drivetrain);

    frc2::CommandPtr moveBackwardsCommand(Drivetrain *c_drivetrain);
    frc2::CommandPtr moveForwardsCommand(Drivetrain *c_drivetrain);

}
