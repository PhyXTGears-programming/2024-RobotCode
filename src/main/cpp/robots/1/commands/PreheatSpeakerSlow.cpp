#include "robots/1/commands/PreheatSpeakerSlow.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace robot1;

robot1::PreheatSpeakerSlow::PreheatSpeakerSlow(SpeakerShooterSubsystem * speaker) {
    AddRequirements(speaker);

    m_speaker = speaker;
}

void robot1::PreheatSpeakerSlow::Initialize() {}

void robot1::PreheatSpeakerSlow::Execute() {
    m_speaker->Shoot();
}

void robot1::PreheatSpeakerSlow::End(bool interrupted) {}

bool robot1::PreheatSpeakerSlow::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_speaker->GetSlowSpeedThreshold();
}
