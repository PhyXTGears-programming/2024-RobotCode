#include "robots/2/commands/ShootSpeaker.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot2;

robot2::ShootSpeaker::ShootSpeaker(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    AddRequirements(intake);
    AddRequirements(speaker);

    m_intake = intake;
    m_speaker = speaker;
}

void robot2::ShootSpeaker::Initialize() {}

void robot2::ShootSpeaker::Execute() {
    m_speaker->Shoot();
    m_intake->IntakeSpeakerShooter();
}

void robot2::ShootSpeaker::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool robot2::ShootSpeaker::IsFinished() {
    return false;
}
