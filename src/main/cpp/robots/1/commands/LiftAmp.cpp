#include "robots/1/commands/LiftAmp.h"
#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

using namespace ::robot1;

robot1::LiftAmp::LiftAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void robot1::LiftAmp::Initialize() {
}

void robot1::LiftAmp::Execute() {
    m_amp->Lift();
}

void robot1::LiftAmp::End(bool interrupted) {
}

bool robot1::LiftAmp::IsFinished() {
    return false;
}
