#pragma once

#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc/XboxController.h>

namespace robot1::diagnostic {
    using namespace ::robot1;

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
