#include "commands/PreheatSpeakerSlow.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

PreheatSpeakerSlow::PreheatSpeakerSlow(SpeakerShooterSubsystem * speaker) {
    AddRequirements(speaker);

    m_speaker = speaker;
}

void PreheatSpeakerSlow::Initialize() {}

void PreheatSpeakerSlow::Execute() {
    m_speaker->Shoot();
}

void PreheatSpeakerSlow::End(bool interrupted) {}

bool PreheatSpeakerSlow::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_speaker->GetSlowSpeedThreshold();
}
