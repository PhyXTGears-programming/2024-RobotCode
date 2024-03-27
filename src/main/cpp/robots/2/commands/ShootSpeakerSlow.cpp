#include "robots/2/commands/ShootSpeakerSlow.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

robot2::ShootSpeakerSlow::ShootSpeakerSlow(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    AddRequirements(intake);
    AddRequirements(speaker);

    m_intake = intake;
    m_speaker = speaker;
}

void robot2::ShootSpeakerSlow::Initialize() {}

void robot2::ShootSpeakerSlow::Execute() {
    m_speaker->SlowShoot();
    m_intake->IntakeSpeakerShooter();
}

void robot2::ShootSpeakerSlow::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool robot2::ShootSpeakerSlow::IsFinished() {
    return false;
}
