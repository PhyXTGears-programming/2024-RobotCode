#pragma once

#include "robots/2/subsystems/intake/Intake.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"
#include <frc/XboxController.h>

namespace robot2::diagnostic {
    using namespace ::robot2;

    class TestIntake {
        public:
            TestIntake(
                IntakeSubsystem * intake,
                SpeakerShooterSubsystem * speaker,
                frc::XboxController * controller
            );

            void Test01TuneIntake();

            IntakeSubsystem * m_intake = nullptr;
            SpeakerShooterSubsystem * m_speaker = nullptr;

            frc::XboxController * m_controller = nullptr;
    };
}
