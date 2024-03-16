#pragma once

#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class PreheatSpeakerSlow : public frc2::CommandHelper<frc2::Command, PreheatSpeakerSlow> {
    public:
        PreheatSpeakerSlow(SpeakerShooterSubsystem * speaker);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        SpeakerShooterSubsystem * m_speaker = nullptr;
};