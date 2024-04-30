#pragma once

#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class PreheatIndefinite: public frc2::CommandHelper<frc2::Command, PreheatIndefinite> {
        public:
            PreheatIndefinite(SpeakerShooterSubsystem * speaker, bool &stopPreheat);

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            SpeakerShooterSubsystem * m_speaker = nullptr;
            bool &m_stopPreheat;
    };

}
