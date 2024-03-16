#pragma once

#include "subsystems/intake/Intake.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class ShootSpeakerSlow : public frc2::CommandHelper<frc2::Command, ShootSpeakerSlow> {
    public:
        ShootSpeakerSlow(
            IntakeSubsystem * intake,
            SpeakerShooterSubsystem * speaker
        );

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        IntakeSubsystem * m_intake = nullptr;
        SpeakerShooterSubsystem * m_speaker = nullptr;
};