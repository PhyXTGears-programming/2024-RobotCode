
#include "commands/IntakeAmp.h"

IntakeAmp::IntakeAmp(IntakeSubsystem * intake, AmpShooterSubsystem * amp) {
    AddRequirements(intake);
    AddRequirements(amp);

    m_amp = amp;
    m_intake = intake;
}

void IntakeAmp::Initialize() {

}

void IntakeAmp::Execute() {
    m_intake->IntakeAmpShooter();
}

void IntakeAmp::End(bool interrupted) {
    m_intake->Stop();
}

bool IntakeAmp::IsFinished() {
    return m_amp->IsNoteDetected();
}