#include "robots/1/auto/auto.h"
#include "robots/1/auto/load/command/Command.h"
#include "robots/1/commands/FollowPath.h"

#include <optional>

using namespace ::robot1;

std::vector<frc::Pose2d> robot1::loadPosePathFromJSON(wpi::json &json) {
    std::vector<frc::Pose2d> path = {};

    for (int i = 0; i < json.size(); i++) {
        wpi::json pathPoint = json[i];

        double x   = pathPoint.value("x",   0.0);
        double y   = pathPoint.value("y",   0.0);
        double rot = pathPoint.value("rot", 0.0);

        frc::Pose2d pose{units::meter_t{x}, units::meter_t{y}, units::radian_t{rot}};

        path.push_back(pose);
    }

    return path;
}

std::vector<PathPoint> robot1::loadPathFromJSON(wpi::json &json, SubsystemRegistry & registry) {
    std::vector<PathPoint> path = {};

    for (int i = 0; i < json.size(); i++) {
        wpi::json pathPoint = json[i];

        double x   = pathPoint.value("x",   0.0);
        double y   = pathPoint.value("y",   0.0);
        double rot = pathPoint.value("rot", 0.0);
        double vel = pathPoint.value("vel", 0.0);

        PathPointType pointType = TYPE_OTHER;
        if ("stop" == pathPoint.value("type", "")) {
            pointType = TYPE_HALT;
        }

        PathPoint point{units::meter_t{x}, units::meter_t{y}, units::radian_t{rot}, units::meters_per_second_t{vel}};
        point.SetType(pointType);

        if (pathPoint.contains("commands")) {
            auto commands = pathPoint["commands"];

            if (commands.contains("rootNode")) {
                std::optional<frc2::CommandPtr> command = importCommand(commands["rootNode"], registry);

                point.SetCommand(std::move(command));
            }
        }

        path.push_back(std::move(point));
    }

    return path;
}

frc2::CommandPtr robot1::loadPoseFollowCommandFromFile(Drivetrain *m_drivetrain, std::string_view filename) {
    std::cout << std::endl << "Robot: building path for auto from '" << filename << "'" << std::endl;
    try {
        std::error_code ec;
        std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
            wpi::MemoryBuffer::GetFile(
                filename,
                ec
            );

        if (nullptr == fileBuffer || ec) {
            std::cerr << "Error: Robot: unable to load path json" << std::endl;
            abort();
        } else {
            wpi::json json = wpi::json::parse(fileBuffer->begin(), fileBuffer->end());

            auto path = loadPosePathFromJSON(json);
            std::cout << std::endl << "Robot: auto path loaded from '" << filename << "'" << std::endl;

            return generatePathFollowCommand(path, 1.5_mps, m_drivetrain);
        }
    } catch (...) {
        std::cerr << "Error: Robot: unknown exception while configuring path" << std::endl;
    }

    return frc2::cmd::None();
}

frc2::CommandPtr robot1::loadPathFollowCommandFromFile(std::string_view filename, SubsystemRegistry & registry) {
    std::cout << std::endl << "Robot: building path for auto from '" << filename << "'" << std::endl;
    try {
        std::error_code ec;
        std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
            wpi::MemoryBuffer::GetFile(
                filename,
                ec
            );

        if (nullptr == fileBuffer || ec) {
            std::cerr << "Error: Robot: unable to load path json from file '" << filename << "'" << std::endl;
            abort();
        } else {
            wpi::json json = wpi::json::parse(fileBuffer->begin(), fileBuffer->end());

            std::vector<PathPoint> path = loadPathFromJSON(json, registry);
            std::cout << std::endl << "Robot: auto path loaded from '" << filename << "'" << std::endl;

            return FollowPath(
                std::move(path),
                registry.drivetrain,
                registry.intake,
                registry.speaker
            ).ToPtr();
        }
    } catch (...) {
        std::cerr << "Error: Robot: unknown exception while configuring path" << std::endl;
    }

    std::cout << std::endl << "Robot: skipping auto json load. using cmd::None'" << std::endl;

    return frc2::cmd::None();
}

#define MAX_PATH_POSE_DISTANCE  0.08_m
#define ROTATION_DEAD_ZONE      DEG_2_RAD(5) // Radians
#define MAX_ROTATION_SPEED      M_PI / 2.0 // Radians per second
#define MIN_ROTATION_SPEED      M_PI / 8.0 // Radians per second
#define HALT_DISTANCE_THRESHOLD 0.05 // Meters
#define MIN_SPEED               0.375_mps
frc2::CommandPtr robot1::generatePathFollowCommand(std::vector<frc::Pose2d> path, units::meters_per_second_t speed, Drivetrain *c_drivetrain) {
    int    *currentPoseIndex  = new int(0);
    Vector *movementDirection = new Vector(0, 0);


    return frc2::FunctionalCommand{
        [=]() { // Initializer - Start of command
            *currentPoseIndex = 0;
            *movementDirection = Vector(0, 0);

            c_drivetrain->SetPosition(c_drivetrain->GetHeading(), path[0]);
        },
        [=]() { // Execute - Every run of command
            Point currentPoint = c_drivetrain->GetChassisPosition();

            for (int i = *currentPoseIndex; i < path.size(); i++) {
                frc::Pose2d pose = path[i];

                units::meter_t distance = units::meter_t{sqrt(pow(pose.X().value() - currentPoint.x, 2.0) + pow(pose.Y().value() - currentPoint.y, 2.0))};

                if (distance > MAX_PATH_POSE_DISTANCE) {
                    *currentPoseIndex = i;
                    break;
                }

                if (i == path.size() - 1) {
                    *currentPoseIndex = i;
                }
            }

            // Movement
            frc::Pose2d selectedPose = path[*currentPoseIndex];

            Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());

            Vector directionToTarget = (targetPoint - currentPoint).unit();
            double distanceToTarget  = (targetPoint - currentPoint).len();

            *movementDirection = directionToTarget;

            // Rotation
            double currentHeading = c_drivetrain->GetHeading().value();

            double targetRotationRadians = selectedPose.Rotation().Radians().value();
            double headingDelta = std::remainder(targetRotationRadians - currentHeading, M_PI * 2.0);

            double rotationSpeed = 0.0;
            if (abs(headingDelta) > ROTATION_DEAD_ZONE) {
                rotationSpeed = std::copysign(std::clamp(std::abs(headingDelta) / std::numbers::pi, 0.05, 1.0), headingDelta) * MAX_ROTATION_SPEED;
            }
            units::radians_per_second_t rotationSpeedRadians{rotationSpeed};

            c_drivetrain->Drive(movementDirection->x * speed, movementDirection->y * speed, rotationSpeedRadians, true, 20_ms);
        },
        [=](bool done) { // End - On command finish
            c_drivetrain->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true, 20_ms);
        },
        [=]() { // Is Finished - Returns true if the command should be done
            if (*currentPoseIndex != path.size() - 1) return false;

            Point currentPoint = c_drivetrain->GetChassisPosition();
            frc::Pose2d selectedPose = path[*currentPoseIndex];
            Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());
            double distanceToTarget = (targetPoint - currentPoint).len();

            return distanceToTarget < 0.05;
        },
        { c_drivetrain }
    }.ToPtr();
}

frc2::CommandPtr robot1::generatePathFollowCommand(std::vector<PathPoint> && path, Drivetrain *c_drivetrain) {
    int    *currentPoseIndex  = new int(0);
    Vector *movementDirection = new Vector(0, 0);
    int    *haltPointIndex    = new int(-1);

    return frc2::FunctionalCommand{
        [=, &path]() { // Initializer - Start of command
            *currentPoseIndex = 0;
            *movementDirection = Vector(0, 0);
            *haltPointIndex = -1;

            c_drivetrain->SetPosition(c_drivetrain->GetHeading(), path[0].Pose());
        },
        [=, &path]() { // Execute - Every run of command
            Point currentPoint = c_drivetrain->GetChassisPosition();

            if (*haltPointIndex == -1) {
                // No halt point selected.

                // Search for next appealing point.
                for (int i = *currentPoseIndex; i < path.size(); i++) {
                    PathPoint & pose = path[i];

                    units::meter_t distance = units::meter_t{sqrt(pow(pose.X().value() - currentPoint.x, 2.0) + pow(pose.Y().value() - currentPoint.y, 2.0))};

                    if (pose.Type() == TYPE_HALT && i != *currentPoseIndex) {
                        // Found halt point that is not the current point.
                        *haltPointIndex = i;
                        *currentPoseIndex = i;

                        // Break early so we don't look past this stop waypoint.
                        break;
                    }

                    if (distance > MAX_PATH_POSE_DISTANCE) {
                        // Found point just outside max distance.
                        *currentPoseIndex = i;
                        break;
                    }

                    if (i == path.size() - 1) {
                        // No appealing point found before the end of the list.
                        // Use the last point.
                        *currentPoseIndex = i;
                    }
                }
            } else {
                // Halt point already selected.  Maintain approach to halt point.
                *currentPoseIndex = *haltPointIndex;
            }

            // Movement
            const PathPoint & selectedPose = path[*currentPoseIndex];

            Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());

            Vector directionToTarget = (targetPoint - currentPoint).unit();
            double distanceToTarget  = (targetPoint - currentPoint).len();

            *movementDirection = directionToTarget;

            units::meters_per_second_t speed = selectedPose.Velocity();
            if (*haltPointIndex == -1) {
                if (distanceToTarget < HALT_DISTANCE_THRESHOLD) {
                    // Run point commands
                    *haltPointIndex = -1;
                } else {
                    speed = MIN_SPEED;
                }
            }

            // Rotation
            double currentHeading = c_drivetrain->GetHeading().value();

            double targetRotationRadians = selectedPose.Rotation().Radians().value();
            double headingDelta = std::remainder(targetRotationRadians - currentHeading, M_PI * 2.0);

            double rotationSpeed = 0.0;
            if (abs(headingDelta) > ROTATION_DEAD_ZONE) {
                rotationSpeed = std::copysign(std::clamp(std::abs(headingDelta) / std::numbers::pi, MIN_ROTATION_SPEED / MAX_ROTATION_SPEED, 1.0), headingDelta) * MAX_ROTATION_SPEED;
            }
            units::radians_per_second_t rotationSpeedRadians{rotationSpeed};

            c_drivetrain->Drive(movementDirection->x * speed, movementDirection->y * speed, rotationSpeedRadians, true, 20_ms);
        },
        [=](bool done) { // End - On command finish
            c_drivetrain->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true, 20_ms);
        },
        [=, &path]() { // Is Finished - Returns true if the command should be done
            if (*currentPoseIndex != path.size() - 1) return false;

            Point currentPoint = c_drivetrain->GetChassisPosition();
            const PathPoint & selectedPose = path[*currentPoseIndex];
            Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());
            double distanceToTarget = (targetPoint - currentPoint).len();

            return distanceToTarget < 0.05;
        },
        { c_drivetrain }
    }.ToPtr();
}

frc2::CommandPtr robot1::moveBackwardsCommand(Drivetrain *c_drivetrain) {
    std::vector<frc::Pose2d> path{
        frc::Pose2d(-0.0_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.1_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.2_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.3_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.4_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.5_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.6_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.7_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.8_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-0.9_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-1.0_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-1.1_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-1.2_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-1.3_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-1.4_m, 0.0_m, 0.0_rad),
        frc::Pose2d(-1.5_m, 0.0_m, 0.0_rad),

    };
    
    return generatePathFollowCommand(path, 1_mps, c_drivetrain);
}

frc2::CommandPtr robot1::moveForwardsCommand(Drivetrain *c_drivetrain) {
    std::vector<frc::Pose2d> path{
        frc::Pose2d(0.0_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.1_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.2_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.3_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.4_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.5_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.6_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.7_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.8_m, 0.0_m, 0.0_rad),
        frc::Pose2d(0.9_m, 0.0_m, 0.0_rad),
        frc::Pose2d(1.0_m, 0.0_m, 0.0_rad),
        frc::Pose2d(1.1_m, 0.0_m, 0.0_rad),
        frc::Pose2d(1.2_m, 0.0_m, 0.0_rad),
        frc::Pose2d(1.3_m, 0.0_m, 0.0_rad),
        frc::Pose2d(1.4_m, 0.0_m, 0.0_rad),
        frc::Pose2d(1.5_m, 0.0_m, 0.0_rad),

    };
    
    return generatePathFollowCommand(path, 1_mps, c_drivetrain);
}
