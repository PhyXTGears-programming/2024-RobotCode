#pragma once

#include "subsystems/amp_shooter/AmpShooter.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc/XboxController.h>

namespace diagnostic {
    class TestIntake {
        public:
            TestIntake(
                IntakeSubsystem * intake,
                AmpShooterSubsystem * amp,
                SpeakerShooterSubsystem * speaker,
                frc::XboxController * controller
            );

            void Test01TuneIntake();

            AmpShooterSubsystem * m_amp = nullptr;
            IntakeSubsystem * m_intake = nullptr;
            SpeakerShooterSubsystem * m_speaker = nullptr;

            frc::XboxController * m_controller = nullptr;
    };
}
