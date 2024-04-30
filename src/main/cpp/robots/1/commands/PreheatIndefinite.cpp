#include "robots/1/commands/PreheatIndefinite.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot1;

robot1::PreheatIndefinite::PreheatIndefinite(SpeakerShooterSubsystem * speaker, bool &stopPreheat) : m_speaker(speaker), m_stopPreheat(stopPreheat) {
    AddRequirements(speaker);
}

void robot1::PreheatIndefinite::Initialize() {
    m_stopPreheat = false;
}

void robot1::PreheatIndefinite::Execute() {
    m_speaker->Shoot();
}

void robot1::PreheatIndefinite::End(bool interrupted) {}

bool robot1::PreheatIndefinite::IsFinished() {
    return m_stopPreheat;
}
