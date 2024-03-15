#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>

#include <frc/geometry/Pose2d.h>

#include <units/velocity.h>

#include "subsystems/drivetrain/Drivetrain.h"

#include "util/vector.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "util/geom.h"

frc2::CommandPtr generatePathFollowCommand(std::vector<frc::Pose2d> path, units::meters_per_second_t speed, Drivetrain *c_drivetrain);

frc2::CommandPtr moveBackwardsCommand(Drivetrain *c_drivetrain);