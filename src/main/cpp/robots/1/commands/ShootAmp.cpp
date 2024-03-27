#include "robots/1/commands/ShootAmp.h"
#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

using namespace robot1;

robot1::ShootAmp::ShootAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void robot1::ShootAmp::Initialize() {
}

void robot1::ShootAmp::Execute() {
    m_amp->Shoot();
}

void robot1::ShootAmp::End(bool interrupted) {
    m_amp->StopShoot();
}

bool robot1::ShootAmp::IsFinished() {
    return false;
}
