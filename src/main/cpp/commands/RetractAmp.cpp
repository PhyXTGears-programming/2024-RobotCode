#include "commands/RetractAmp.h"
#include "subsystems/amp_shooter/AmpShooter.h"

RetractAmp::RetractAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void RetractAmp::Initialize() {
m_amp->Retract();
}

void RetractAmp::Execute() {
}

void RetractAmp::End(bool interrupted) {
}

bool RetractAmp::IsFinished() {
    return true;
}
