#include "commands/ShootSpeaker.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

ShootSpeaker::ShootSpeaker(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    AddRequirements(intake);
    AddRequirements(speaker);

    m_intake = intake;
    m_speaker = speaker;
}

void ShootSpeaker::Initialize() {}

void ShootSpeaker::Execute() {
    m_speaker->Shoot();
    m_intake->IntakeSpeakerShooter();
}

void ShootSpeaker::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool ShootSpeaker::IsFinished() {
    return false;
}