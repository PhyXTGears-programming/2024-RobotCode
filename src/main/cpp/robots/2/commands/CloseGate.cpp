#include "robots/2/commands/CloseGate.h"

using namespace ::robot2;

robot2::CloseGate::CloseGate(GateSubsystem * gate) {
    AddRequirements(gate);

    m_gate = gate;
}

void robot2::CloseGate::Initialize() {
    m_gate->Close();
}

void robot2::CloseGate::Execute() {}

void robot2::CloseGate::End(bool interrupted) {}

bool robot2::CloseGate::IsFinished() {
    return true;
}
