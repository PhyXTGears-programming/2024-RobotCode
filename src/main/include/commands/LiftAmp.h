#pragma once

#include "subsystems/amp_shooter/AmpShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class LiftAmp : public frc2::CommandHelper<frc2::Command, LiftAmp> {
    public:
        LiftAmp(AmpShooterSubsystem * amp);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;
        
    private:
        AmpShooterSubsystem * m_amp = nullptr;
};
