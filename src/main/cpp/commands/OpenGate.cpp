#include "commands/OpenGate.h"

OpenGate::OpenGate(GateSubsystem * gate) {
    AddRequirements(gate);

    m_gate = gate;
}

void OpenGate::Initialize() {
    m_gate->Open();
}

void OpenGate::Execute() {}

void OpenGate::End(bool interrupted) {}

bool OpenGate::IsFinished() {
    return true;
}