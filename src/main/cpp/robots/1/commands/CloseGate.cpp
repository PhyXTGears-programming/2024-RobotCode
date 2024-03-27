#include "robots/1/commands/CloseGate.h"

using namespace robot1;

robot1::CloseGate::CloseGate(GateSubsystem * gate) {
    AddRequirements(gate);

    m_gate = gate;
}

void robot1::CloseGate::Initialize() {
    m_gate->Close();
}

void robot1::CloseGate::Execute() {}

void robot1::CloseGate::End(bool interrupted) {}

bool robot1::CloseGate::IsFinished() {
    return true;
}
