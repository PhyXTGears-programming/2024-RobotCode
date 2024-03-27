#pragma once

#include "robots/1/subsystems/intake/Intake.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class ShootSpeaker : public frc2::CommandHelper<frc2::Command, ShootSpeaker> {
        public:
            ShootSpeaker(
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

}
