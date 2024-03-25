#pragma once

#include "auto/PathPoint.h"
#include "subsystems/drivetrain/Drivetrain.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <queue>
#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

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
        bool m_shallHaltForCommand;
};
