#pragma once

#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class ExtendAmp : public frc2::CommandHelper<frc2::Command, ExtendAmp> {
        public:
            ExtendAmp(AmpShooterSubsystem * amp);

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            AmpShooterSubsystem * m_amp = nullptr;
    };

}
