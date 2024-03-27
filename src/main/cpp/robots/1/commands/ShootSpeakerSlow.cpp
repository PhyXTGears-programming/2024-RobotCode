#include "robots/1/commands/ShootSpeakerSlow.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

robot1::ShootSpeakerSlow::ShootSpeakerSlow(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    AddRequirements(intake);
    AddRequirements(speaker);

    m_intake = intake;
    m_speaker = speaker;
}

void robot1::ShootSpeakerSlow::Initialize() {}

void robot1::ShootSpeakerSlow::Execute() {
    m_speaker->SlowShoot();
    m_intake->IntakeSpeakerShooter();
}

void robot1::ShootSpeakerSlow::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool robot1::ShootSpeakerSlow::IsFinished() {
    return false;
}
