#include "robots/2/commands/OpenGate.h"

using namespace ::robot2;

robot2::OpenGate::OpenGate(GateSubsystem * gate) {
    AddRequirements(gate);

    m_gate = gate;
}

void robot2::OpenGate::Initialize() {
    m_gate->Open();
}

void robot2::OpenGate::Execute() {}

void robot2::OpenGate::End(bool interrupted) {}

bool robot2::OpenGate::IsFinished() {
    return true;
}
