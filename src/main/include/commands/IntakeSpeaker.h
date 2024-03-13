#pragma once

#include "subsystems/speaker_shooter/SpeakerShooter.h"
#include "subsystems/intake/Intake.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class IntakeSpeaker : public frc2::CommandHelper<frc2::CommandBase, IntakeSpeaker> {
    public:
        IntakeSpeaker(IntakeSubsystem * intake,  SpeakerShooterSubsystem  * amp);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        SpeakerShooterSubsystem * m_speaker = nullptr;
        IntakeSubsystem * m_intake = nullptr;
};