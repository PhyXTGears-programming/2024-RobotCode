#include "robots/2/commands/Shoot.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot2;

robot2::Shoot::Shoot(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker,
    rpm_t speed,
    units::volt_t feedForward,
    units::microsecond_t leftMicros,
    units::microsecond_t rightMicros
) :
    m_intake(intake),
    m_speaker(speaker),
    m_speed(speed),
    m_feedForward(feedForward),
    m_leftMicros(leftMicros),
    m_rightMicros(rightMicros)
{
    AddRequirements(intake);
    AddRequirements(speaker);
}

void robot2::Shoot::Initialize() {}

void robot2::Shoot::Execute() {
    m_speaker->SetShooterSpeed(m_speed, m_feedForward);
    m_intake->IntakeSpeakerShooter();
}

void robot2::Shoot::End(bool interrupted) {
    m_intake->Stop();
    m_speaker->StopShooter();
}

bool robot2::Shoot::IsFinished() {
    return false;
}
