#pragma once

#include "subsystems/bling/Bling.h"
#include "subsystems/climb/Climb.h"

#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class ClimbUp : public frc2::CommandHelper<frc2::Command, ClimbUp> {
    public:
        ClimbUp(
            ClimbSubsystem * climb, 
            BlingSubsystem * bling,
            frc::XboxController * controller
        );

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        ClimbSubsystem * m_climb = nullptr;
        BlingSubsystem * m_bling = nullptr;

        frc::XboxController * m_controller = nullptr;
};