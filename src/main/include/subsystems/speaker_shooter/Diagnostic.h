#pragma once

#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc/XboxController.h>

namespace diagnostic {
    class TestSpeaker {
        public:
            TestSpeaker(
                SpeakerShooterSubsystem * speaker,
                frc::XboxController * controller
            );

            void Test01TuneSpeaker();

            SpeakerShooterSubsystem * m_speaker = nullptr;

            frc::XboxController * m_controller = nullptr;
    };
}