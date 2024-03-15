#pragma once

#include "subsystems/amp_shooter/AmpShooter.h"
#include "subsystems/intake/Intake.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class IntakeAmp : public frc2::CommandHelper<frc2::Command, IntakeAmp> {
    public:
        IntakeAmp(IntakeSubsystem * intake, AmpShooterSubsystem * amp);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        AmpShooterSubsystem * m_amp = nullptr;
        IntakeSubsystem * m_intake = nullptr;
};