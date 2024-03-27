#pragma once

#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc/XboxController.h>

namespace robot2::diagnostic {
    using namespace ::robot2;

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
