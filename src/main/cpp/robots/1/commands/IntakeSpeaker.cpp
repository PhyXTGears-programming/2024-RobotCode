#include "robots/1/commands/IntakeSpeaker.h"

using namespace ::robot1;

robot1::IntakeSpeaker::IntakeSpeaker(IntakeSubsystem * intake,  SpeakerShooterSubsystem  * speaker) {
    AddRequirements(intake); 
    AddRequirements(speaker);

    m_speaker = speaker;
    m_intake = intake;
}

void robot1::IntakeSpeaker::Initialize() {

}

void robot1::IntakeSpeaker::Execute() {
    m_intake->IntakeSpeakerShooter();
}

void robot1::IntakeSpeaker::End(bool interrupted) {
    m_intake->Stop();
}

bool robot1::IntakeSpeaker::IsFinished() {
    return m_speaker->IsNoteDetected();
}
