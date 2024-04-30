#include "robots/1/commands/OpenGate.h"

using namespace ::robot1;

robot1::OpenGate::OpenGate(GateSubsystem * gate) {
    AddRequirements(gate);

    m_gate = gate;
}

void robot1::OpenGate::Initialize() {
    m_gate->Open();
}

void robot1::OpenGate::Execute() {}

void robot1::OpenGate::End(bool interrupted) {}

bool robot1::OpenGate::IsFinished() {
    return true;
}
