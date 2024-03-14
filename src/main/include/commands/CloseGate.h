#pragma once


#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gate/Gate.h"

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