#include "commands/CloseGate.h"

CloseGate::CloseGate(GateSubsystem * gate) {
    AddRequirements(gate);

    m_gate = gate;
}

void CloseGate::Initialize() {
    m_gate->Close();
}

void CloseGate::Execute() {}

void CloseGate::End(bool interrupted) {}

bool CloseGate::IsFinished() {
    return true;
}