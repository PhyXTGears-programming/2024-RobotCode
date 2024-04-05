#include "robots/2/commands/Shoot.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot2;

robot2::Shoot::Shoot(
    SpeakerShooterSubsystem * speaker,
    rpm_t speed,
    units::volt_t feedForward,
    double tilt
) :
    m_speaker(speaker),
    m_speed(speed),
    m_feedForward(feedForward),
    m_tilt(std::clamp(tilt, 0.0, 0.8))
{
    AddRequirements(speaker);
}

void robot2::Shoot::Initialize() {}

void robot2::Shoot::Execute() {
    m_speaker->SetTilt(m_tilt);
    m_speaker->SetShooterSpeed(m_speed, m_feedForward);
    m_speaker->FeedNote();
}

void robot2::Shoot::End(bool interrupted) {
    m_speaker->StopFeed();
    m_speaker->StopShooter();
}

bool robot2::Shoot::IsFinished() {
    return false;
}
