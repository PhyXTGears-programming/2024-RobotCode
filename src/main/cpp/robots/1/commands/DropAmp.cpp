#include "robots/1/commands/DropAmp.h"
#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

using namespace robot1;

robot1::DropAmp::DropAmp(AmpShooterSubsystem * amp) {
    AddRequirements(amp);

    m_amp = amp;
}

void robot1::DropAmp::Initialize() {
}

void robot1::DropAmp::Execute() {
}

void robot1::DropAmp::End(bool interrupted) {
}

bool robot1::DropAmp::IsFinished() {
}
