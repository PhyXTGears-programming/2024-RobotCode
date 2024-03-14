#include "commands/PreheatSpeaker.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

PreheatSpeaker::PreheatSpeaker(SpeakerShooterSubsystem * speaker) {
    AddRequirements(speaker);

    m_speaker = speaker;
}

void PreheatSpeaker::Initialize() {}

void PreheatSpeaker::Execute() {
    m_speaker->Shoot();
}

void PreheatSpeaker::End(bool interrupted) {}

bool PreheatSpeaker::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_speaker->GetSpeedThreshold();
}
