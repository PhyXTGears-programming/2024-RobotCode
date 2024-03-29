#include "robots/2/commands/Commands.h"
#include "robots/2/commands/PreheatShooter.h"
#include "robots/2/commands/Shoot.h"

#include "robots/2/subsystems/intake/Intake.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

using namespace ::robot2;

frc2::CommandPtr robot2::cmd::ShootAmp(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        intake,
        speaker,
        speaker->m_config.amp.shoot.speed,
        speaker->m_config.amp.shoot.feedForward,
        speaker->m_config.amp.tilt
    ).ToPtr();
}


frc2::CommandPtr robot2::cmd::ShootSpeakerFar(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        intake,
        speaker,
        speaker->m_config.speaker.far.shoot.speed,
        speaker->m_config.speaker.far.shoot.feedForward,
        speaker->m_config.speaker.far.tilt
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::ShootSpeakerNear(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        intake,
        speaker,
        speaker->m_config.speaker.near.shoot.speed,
        speaker->m_config.speaker.near.shoot.feedForward,
        speaker->m_config.speaker.near.tilt
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::ShootTrap(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        intake,
        speaker,
        speaker->m_config.trap.shoot.speed,
        speaker->m_config.trap.shoot.feedForward,
        speaker->m_config.trap.tilt
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatAmp(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.amp.shoot.speed,
        speaker->m_config.amp.shoot.feedForward,
        speaker->m_config.amp.shoot.speed,
        speaker->m_config.amp.tilt
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatSpeakerFar(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.speaker.far.shoot.speed,
        speaker->m_config.speaker.far.shoot.feedForward,
        speaker->m_config.speaker.far.shoot.speed,
        speaker->m_config.speaker.far.tilt
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatSpeakerNear(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.speaker.near.shoot.speed,
        speaker->m_config.speaker.near.shoot.feedForward,
        speaker->m_config.speaker.near.shoot.speed,
        speaker->m_config.speaker.near.tilt
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatTrap(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.trap.shoot.speed,
        speaker->m_config.trap.shoot.feedForward,
        speaker->m_config.trap.shoot.speed,
        speaker->m_config.trap.tilt
    ).ToPtr();
}
