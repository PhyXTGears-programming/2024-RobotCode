#include "robots/1/commands/IntakeAmp.h"

using namespace robot1;

robot1::IntakeAmp::IntakeAmp(IntakeSubsystem * intake, AmpShooterSubsystem * amp) {
    AddRequirements(intake);
    AddRequirements(amp);

    m_amp = amp;
    m_intake = intake;
}

void robot1::IntakeAmp::Initialize() {

}

void robot1::IntakeAmp::Execute() {
    m_intake->IntakeAmpShooter();
}

void robot1::IntakeAmp::End(bool interrupted) {
    m_intake->Stop();
}

bool robot1::IntakeAmp::IsFinished() {
    return m_amp->IsNoteDetected();
}
