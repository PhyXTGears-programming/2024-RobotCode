#include "commands/ShootSpeakerSlow.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

ShootSpeakerSlow::ShootSpeakerSlow(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    AddRequirements(intake);
    AddRequirements(speaker);

    m_intake = intake;
    m_speaker = speaker;
}

void ShootSpeakerSlow::Initialize() {}

void ShootSpeakerSlow::Execute() {
    m_speaker->SlowShoot();
    m_intake->IntakeSpeakerShooter();
}

void ShootSpeakerSlow::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool ShootSpeakerSlow::IsFinished() {
    return false;
}