#include "robots/1/commands/ExtendAmp.h"
#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

using namespace robot1;

robot1::ExtendAmp::ExtendAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void robot1::ExtendAmp::Initialize() {
    m_amp->Extend();
}

void robot1::ExtendAmp::Execute() {
}

void robot1::ExtendAmp::End(bool interrupted) {
}

bool robot1::ExtendAmp::IsFinished() {
    return true;
}
