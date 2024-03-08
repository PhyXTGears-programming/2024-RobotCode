#include "commands/LiftAmp.h"
#include "subsystems/amp_shooter/AmpShooter.h"

LiftAmp::LiftAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void LiftAmp::Initialize() {
}

void LiftAmp::Execute() {
m_amp->Lift();
}

void LiftAmp::End(bool interrupted) {
}

bool LiftAmp::IsFinished() {
return false;
}
