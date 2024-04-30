#pragma once

#include "robots/1/subsystems/gate/Gate.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class CloseGate : public frc2::CommandHelper<frc2::Command, CloseGate> {
    public:
        CloseGate(GateSubsystem * gate);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        GateSubsystem * m_gate;
    };

}
