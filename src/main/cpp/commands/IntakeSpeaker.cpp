
#include "commands/IntakeSpeaker.h"

IntakeSpeaker::IntakeSpeaker(IntakeSubsystem * intake,  SpeakerShooterSubsystem  * speaker) {
    AddRequirements(intake); 
    AddRequirements(speaker);

    m_speaker = speaker;
    m_intake = intake;

    
}

void IntakeSpeaker::Initialize() {

}

void IntakeSpeaker::Execute() {
    m_intake->IntakeSpeakerShooter();
}

void IntakeSpeaker::End(bool interrupted) {
    m_intake->Stop();
}

bool IntakeSpeaker::IsFinished() {
    return m_speaker->IsNoteDetected();
}