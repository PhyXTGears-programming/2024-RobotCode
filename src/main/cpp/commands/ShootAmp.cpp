#include "commands/ShootAmp.h"
#include "subsystems/amp_shooter/AmpShooter.h"

ShootAmp::ShootAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void ShootAmp::Initialize() {
}

void ShootAmp::Execute() {
    m_amp->Shoot();
}

void ShootAmp::End(bool interrupted) {
    m_amp->StopShoot();
}

bool ShootAmp::IsFinished() {
    return false;
}
