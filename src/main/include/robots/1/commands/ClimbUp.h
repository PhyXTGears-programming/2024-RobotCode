#pragma once

#include "robots/1/subsystems/bling/Bling.h"
#include "robots/1/subsystems/climb/Climb.h"

#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

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

}
