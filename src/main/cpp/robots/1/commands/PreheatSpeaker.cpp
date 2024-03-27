#include "robots/1/commands/PreheatSpeaker.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace robot1;

robot1::PreheatSpeaker::PreheatSpeaker(SpeakerShooterSubsystem * speaker) {
    AddRequirements(speaker);

    m_speaker = speaker;
}

void robot1::PreheatSpeaker::Initialize() {}

void robot1::PreheatSpeaker::Execute() {
    m_speaker->Shoot();
}

void robot1::PreheatSpeaker::End(bool interrupted) {}

bool robot1::PreheatSpeaker::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_speaker->GetFastSpeedThreshold();
}
