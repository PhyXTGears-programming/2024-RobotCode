#include "robots/2/commands/IntakeSpeaker.h"

using namespace ::robot2;

robot2::IntakeSpeaker::IntakeSpeaker(IntakeSubsystem * intake,  SpeakerShooterSubsystem  * speaker) {
    AddRequirements(intake); 
    AddRequirements(speaker);

    m_speaker = speaker;
    m_intake = intake;
}

void robot2::IntakeSpeaker::Initialize() {

}

void robot2::IntakeSpeaker::Execute() {
    m_intake->IntakeSpeakerShooter();
}

void robot2::IntakeSpeaker::End(bool interrupted) {
    m_intake->Stop();
}

bool robot2::IntakeSpeaker::IsFinished() {
    return m_speaker->IsNoteDetected();
}
