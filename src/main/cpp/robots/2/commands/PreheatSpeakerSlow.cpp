#include "robots/2/commands/PreheatSpeakerSlow.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot2;

robot2::PreheatSpeakerSlow::PreheatSpeakerSlow(SpeakerShooterSubsystem * speaker) {
    AddRequirements(speaker);

    m_speaker = speaker;
}

void robot2::PreheatSpeakerSlow::Initialize() {}

void robot2::PreheatSpeakerSlow::Execute() {
    m_speaker->Shoot();
}

void robot2::PreheatSpeakerSlow::End(bool interrupted) {}

bool robot2::PreheatSpeakerSlow::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_speaker->GetSlowSpeedThreshold();
}
