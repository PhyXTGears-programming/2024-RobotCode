#include "commands/DropAmp.h"
#include "subsystems/amp_shooter/AmpShooter.h"

DropAmp::DropAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void DropAmp::Initialize() {
}

void DropAmp::Execute() {
}

void DropAmp::End(bool interrupted) {
}

bool DropAmp::IsFinished() {
}
