#include "robots/2/commands/FollowPath.h"
#include "util/geom.h"

#include <iostream>
#include <numbers>

#define MAX_PATH_POSE_DISTANCE  6_in
#define ROTATION_DEAD_ZONE      DEG_2_RAD(5) // Radians
#define ROTATION_SPEED          M_PI_2 // Radians per second
#define HALT_DISTANCE_THRESHOLD 0.05 // Meters
#define MIN_SPEED               0.375_mps

#define NEARBY_DISTANCE_THRESHOLD 12_in

using namespace ::robot2;

robot2::FollowPath::FollowPath(
    std::vector<PathPoint> && path,
    Drivetrain * drivetrain,
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) :
    m_drivetrain(drivetrain),
    m_path(std::move(path)),
    m_currentPoseIndex(0),
    m_haltPoseIndex(-1),
    m_lastNearestPoseIndex(0),
    m_cmdQueue(),
    m_isAtHaltPose(false)
{
    AddRequirements({ drivetrain, intake, speaker });
}

void robot2::FollowPath::Initialize() {
    m_currentPoseIndex = 0;
    m_haltPoseIndex = -1;
    m_lastNearestPoseIndex = 0;

    // Make sure the queue is empty.
    while (!m_cmdQueue.empty()) {
        m_cmdQueue.pop();
    }

    m_isAtHaltPose = (TYPE_HALT == m_path[0].Type());

    if(m_path[0].HasCommand()) {
        m_cmdQueue.push(m_path[0].Command());
    }

    if (m_isAtHaltPose && !m_cmdQueue.empty()) {
        m_haltPoseIndex = 0;
    }

    m_drivetrain->SetPosition(m_drivetrain->GetHeading(), m_path[0].Pose());

    if (!m_cmdQueue.empty()) {
        std::cout << "Auto: FollowPath: queue command: pose 0" << std::endl;
        std::cout << "Auto: FollowPath: start command" << std::endl;
        m_cmdQueue.front()->Initialize();
    }
}

void robot2::FollowPath::Execute() {
    Point currentPoint = m_drivetrain->GetChassisPosition();

    // Run the active command if we have one.
    if (!m_cmdQueue.empty()) {
        if (m_cmdQueue.front()->IsFinished()) {
            std::cout << "Auto: FollowPath: end command" << std::endl;
            m_cmdQueue.front()->End(false);
            m_cmdQueue.pop();

            if (!m_cmdQueue.empty()) {
                // Start next command that was queued.
                std::cout << "Auto: FollowPath: start command" << std::endl;
                m_cmdQueue.front()->Initialize();
            } else if ((size_t)-1 != m_haltPoseIndex) {
                // Let path command drive away.
                m_haltPoseIndex = (size_t)-1;
                m_isAtHaltPose = false;
            }
        } else {
            m_cmdQueue.front()->Execute();
        }
    } else if (m_isAtHaltPose) {
        m_haltPoseIndex = (size_t)-1;
        m_isAtHaltPose = false;
    }

    if ((size_t)-1 == m_haltPoseIndex) {
        // No halt point selected.

        // Search for next appealing point.
        // If any new points visited have a halt condition, then look no further.
        for (size_t i = m_currentPoseIndex; i < m_path.size(); i += 1) {
            PathPoint & pose = m_path[i];

            if (TYPE_HALT == pose.Type() && i != m_currentPoseIndex) {
                // Found halt point that is not the current point.
                m_haltPoseIndex = i;
                m_currentPoseIndex = i;
                m_isAtHaltPose = false;

                // Break early so we don't look past this stop waypoint.
                break;
            }

            units::meter_t distance = units::meter_t{
                sqrt(
                    pow(pose.X().value() - currentPoint.x, 2.0)
                    + pow(pose.Y().value() - currentPoint.y, 2.0)
                )
            };

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

    size_t nextNearestPoseIndex = m_lastNearestPoseIndex;

    // If any new nearby points have commands, add those commands to the queue.

    // Collect commands up to current nearest index.
    for (size_t i = m_lastNearestPoseIndex; i <= m_currentPoseIndex && i < m_path.size(); i += 1) {
        PathPoint & pose = m_path[i];

        units::meter_t distance = units::meter_t{
            sqrt(
                pow(pose.X().value() - currentPoint.x, 2.0)
                + pow(pose.Y().value() - currentPoint.y, 2.0)
            )
        };

        if (distance <= NEARBY_DISTANCE_THRESHOLD) {
            nextNearestPoseIndex = i;
        }
    }

    // If new nearby point found, collect all commands since last nearest point.
    for (size_t i = m_lastNearestPoseIndex + 1; i <= nextNearestPoseIndex; i += 1) {
        PathPoint & pose = m_path[i];

        if (pose.HasCommand()) {
            std::cout << "Auto: FollowPath: queue command: pose " << i << std::endl;

            if (m_cmdQueue.empty()) {
                // No commands active.  Run command immediately.
                m_cmdQueue.push(pose.Command());
                std::cout << "Auto: FollowPath: start command" << std::endl;
                m_cmdQueue.front()->Initialize();
            } else {
                // Another command is active.  Only add to queue.
                m_cmdQueue.push(pose.Command());
            }
        }
    }

    m_lastNearestPoseIndex = std::max(m_lastNearestPoseIndex, nextNearestPoseIndex);

    // Movement
    const PathPoint & selectedPose = m_path[m_currentPoseIndex];

    Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());

    Vector vectorToTarget = (targetPoint - currentPoint);
    Vector directionToTarget = vectorToTarget.unit();
    double distanceToTarget = vectorToTarget.len();

    Vector movementDirection = directionToTarget;

    units::meters_per_second_t speed = selectedPose.Velocity();

    if ((size_t)-1 != m_haltPoseIndex) {
        if (distanceToTarget < HALT_DISTANCE_THRESHOLD) {
            // Run point commands.
            m_isAtHaltPose = true;
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

void robot2::FollowPath::End(bool interrupted) {
    m_drivetrain->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true, 20_ms);
}

bool robot2::FollowPath::IsFinished() {
    if (m_currentPoseIndex != m_path.size() - 1) {
        return false;
    }

    Point currentPoint = m_drivetrain->GetChassisPosition();
    const PathPoint & selectedPose = m_path[m_currentPoseIndex];
    Point targetPoint(selectedPose.X().value(), selectedPose.Y().value());
    double distanceToTarget = (targetPoint - currentPoint).len();

    return distanceToTarget < 0.05;
}
