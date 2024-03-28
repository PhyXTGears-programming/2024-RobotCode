#include "robots/2/commands/PreheatShooter.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"
#include <units/math.h>

using namespace ::robot2;

robot2::PreheatShooter::PreheatShooter(
    SpeakerShooterSubsystem * speaker,
    rpm_t speed,
    units::volt_t feedForward,
    rpm_t threshold
) :
    m_speaker(speaker),
    m_speed(speed),
    m_feedForward(feedForward),
    m_threshold(threshold)
{
    AddRequirements(speaker);
}

void robot2::PreheatShooter::Initialize() {}

void robot2::PreheatShooter::Execute() {
    m_speaker->SetShooterSpeed(m_speed, m_feedForward);
}

void robot2::PreheatShooter::End(bool interrupted) {
    m_speaker->StopShooter();
}

bool robot2::PreheatShooter::IsFinished() {
    return m_speaker->GetShooterSpeed() >= m_threshold;
}
