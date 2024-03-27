#include "robots/2/commands/PreheatSpeaker.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot2;

robot2::PreheatSpeaker::PreheatSpeaker(SpeakerShooterSubsystem * speaker) {
    AddRequirements(speaker);

    m_speaker = speaker;
}

void robot2::PreheatSpeaker::Initialize() {}

void robot2::PreheatSpeaker::Execute() {
    m_speaker->Shoot();
}

void robot2::PreheatSpeaker::End(bool interrupted) {}

bool robot2::PreheatSpeaker::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_speaker->GetFastSpeedThreshold();
}
