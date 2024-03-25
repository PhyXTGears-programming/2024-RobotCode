#include "commands/FollowPath.h"
#include "util/geom.h"

#include <iostream>
#include <numbers>

#define MAX_PATH_POSE_DISTANCE  0.08_m
#define ROTATION_DEAD_ZONE      DEG_2_RAD(5) // Radians
#define ROTATION_SPEED          M_PI_2 // Radians per second
#define HALT_DISTANCE_THRESHOLD 0.05 // Meters
#define MIN_SPEED               0.375_mps

FollowPath::FollowPath(
    std::vector<PathPoint> && path,
    Drivetrain * drivetrain,
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) :
    m_drivetrain(drivetrain),
    m_path(std::move(path)),
    m_currentPoseIndex(0),
    m_haltPoseIndex(-1)
{
    AddRequirements({ drivetrain, intake, speaker });
}

void FollowPath::Initialize() {
    m_currentPoseIndex = 0;
    m_haltPoseIndex = -1;

    m_drivetrain->SetPosition(m_drivetrain->GetHeading(), m_path[0].Pose());
}

void FollowPath::Execute() {
    Point currentPoint = m_drivetrain->GetChassisPosition();

    if (-1 == m_haltPoseIndex) {
        // No halt point selected.

        // Search for next appealing point.
        for (int i = m_currentPoseIndex; i < m_path.size(); i += 1) {
            PathPoint pose = m_path[i];

            units::meter_t distance = units::meter_t{
                sqrt(
                    pow(pose.X().value() - currentPoint.x, 2.0)
                    + pow(pose.Y().value() - currentPoint.y, 2.0)
                )
            };

            if (TYPE_HALT == pose.Type() && i != m_currentPoseIndex) {
                // Found halt point that is not the current point.
                m_haltPoseIndex = i;
                m_currentPoseIndex = i;

                // Break early so we don't look past this stop waypoint.
                break;
            }

            if (distance > MAX_PATH_POSE_DISTANCE) {
                // Found point just outside max distance.
                m_currentPoseIndex = i;
                break;
            }

            if (i == m_path.size() - 1) {
                // No appealing point found before the end of the list.
                // Use the last point
                m_currentPoseIndex = i;
            }
        }
    } else {
        // Halt point already selected. Maintain approach to halt point.
        m_currentPoseIndex = m_haltPoseIndex;
    }

    // Movement
    PathPoint selectedPose = m_path[m_currentPoseIndex];

    Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());

    Vector vectorToTarget = (targetPoint - currentPoint);
    Vector directionToTarget = vectorToTarget.unit();
    double distanceToTarget = vectorToTarget.len();

    Vector movementDirection = directionToTarget;

    units::meters_per_second_t speed = selectedPose.Velocity();

    if (-1 == m_haltPoseIndex) {
        if (distanceToTarget < HALT_DISTANCE_THRESHOLD) {
            // Run point commands.
            m_haltPoseIndex = -1;
        } else {
            speed = MIN_SPEED;
        }
    }

    // Rotation
    double currentHeading = m_drivetrain->GetHeading().value();

    double targetRotationRadians = selectedPose.Rotation().Radians().value();
    double headingDelta = std::remainder(
        targetRotationRadians - currentHeading,
        std::numbers::pi * 2.0
    );

    double rotationSpeed = 0.0;

    if (abs(headingDelta) > ROTATION_DEAD_ZONE) {
        rotationSpeed = std::copysign(
            std::clamp(std::abs(headingDelta) / std::numbers::pi, 0.05, 1.0),
            headingDelta
        ) * ROTATION_SPEED;
    }

    units::radians_per_second_t rotationSpeedRadians{rotationSpeed};

    m_drivetrain->Drive(
        movementDirection.x * speed,
        movementDirection.y * speed,
        rotationSpeedRadians,
        true,
        20_ms
    );
}

void FollowPath::End(bool interrupted) {
    m_drivetrain->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true, 20_ms);
}

bool FollowPath::IsFinished() {
    if (m_currentPoseIndex != m_path.size() - 1) {
        return false;
    }

    Point currentPoint = m_drivetrain->GetChassisPosition();
    PathPoint selectedPose = m_path[m_currentPoseIndex];
    Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());
    double distanceToTarget = (targetPoint - currentPoint).len();

    return distanceToTarget < 0.05;
}
