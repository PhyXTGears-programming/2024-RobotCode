#include "robots/1/commands/ShootSpeaker.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace robot1;

robot1::ShootSpeaker::ShootSpeaker(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    AddRequirements(intake);
    AddRequirements(speaker);

    m_intake = intake;
    m_speaker = speaker;
}

void robot1::ShootSpeaker::Initialize() {}

void robot1::ShootSpeaker::Execute() {
    m_speaker->Shoot();
    m_intake->IntakeSpeakerShooter();
}

void robot1::ShootSpeaker::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool robot1::ShootSpeaker::IsFinished() {
    return false;
}
