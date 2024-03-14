#include "commands/ExtendAmp.h"
#include "subsystems/amp_shooter/AmpShooter.h"

ExtendAmp::ExtendAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void ExtendAmp::Initialize() {
    m_amp->Extend();
}

void ExtendAmp::Execute() {
}

void ExtendAmp::End(bool interrupted) {
}

bool ExtendAmp::IsFinished() {
    return true;
}
