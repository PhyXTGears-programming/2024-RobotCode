#pragma once

#include "robots/1/subsystems/amp_shooter/AmpShooter.h"
#include "robots/1/subsystems/intake/Intake.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"
#include <frc/XboxController.h>

namespace robot1::diagnostic {
    using namespace ::robot1;

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
