#pragma once

#include "robots/2/subsystems/gate/Gate.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot2 {

    class OpenGate : public frc2::CommandHelper<frc2::Command, OpenGate> {
    public:
        OpenGate(GateSubsystem * gate);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        GateSubsystem * m_gate;
    };

}
