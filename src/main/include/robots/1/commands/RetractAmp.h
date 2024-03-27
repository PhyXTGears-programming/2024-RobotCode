#pragma once

#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class RetractAmp : public frc2::CommandHelper<frc2::Command, RetractAmp> {
        public:
            RetractAmp(AmpShooterSubsystem * amp);

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            AmpShooterSubsystem * m_amp = nullptr;
    };

}
