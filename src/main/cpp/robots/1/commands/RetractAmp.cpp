#include "robots/1/commands/RetractAmp.h"
#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

using namespace robot1;

robot1::RetractAmp::RetractAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void robot1::RetractAmp::Initialize() {
m_amp->Retract();
}

void robot1::RetractAmp::Execute() {
}

void robot1::RetractAmp::End(bool interrupted) {
}

bool robot1::RetractAmp::IsFinished() {
    return true;
}
