#pragma once

#include "robots/1/auto/PathPoint.h"
#include "robots/1/subsystems/drivetrain/Drivetrain.h"
#include "robots/1/subsystems/intake/Intake.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

#include <queue>
#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class FollowPath : public frc2::CommandHelper<frc2::Command, FollowPath> {
        public:
            FollowPath(
                std::vector<PathPoint> && path,
                Drivetrain * drivetrain,
                IntakeSubsystem * intake,
                SpeakerShooterSubsystem * speaker
            );

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            Drivetrain * m_drivetrain = nullptr;

            std::vector<PathPoint> m_path;

            size_t m_currentPoseIndex;
            size_t m_haltPoseIndex;
            size_t m_lastNearestPoseIndex;

            std::queue<frc2::Command *> m_cmdQueue;
            bool m_isAtHaltPose;
    };

}
