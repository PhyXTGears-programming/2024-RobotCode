#include "robots/2/commands/Commands.h"
#include "robots/2/commands/IntakeSpeaker.h"
#include "robots/2/commands/PreheatShooter.h"
#include "robots/2/commands/Shoot.h"

#include "robots/2/subsystems/intake/Intake.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Commands.h>

using namespace ::robot2;

frc2::CommandPtr robot2::cmd::Intake(
    IntakeSubsystem * intake,
    SpeakerShooterSubsystem * speaker
) {
    return frc2::cmd::Sequence(
        IntakeSpeaker(intake, speaker).ToPtr()
    );
}

frc2::CommandPtr robot2::cmd::ShootAmp(
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        speaker,
        speaker->m_config.amp.shoot.speed,
        speaker->m_config.amp.shoot.feedForward
    ).ToPtr();
}


frc2::CommandPtr robot2::cmd::ShootSpeakerFar(
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        speaker,
        speaker->m_config.speaker.far.shoot.speed,
        speaker->m_config.speaker.far.shoot.feedForward
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::ShootSpeakerNear(
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        speaker,
        speaker->m_config.speaker.near.shoot.speed,
        speaker->m_config.speaker.near.shoot.feedForward
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::ShootTrap(
    SpeakerShooterSubsystem * speaker
) {
    return Shoot(
        speaker,
        speaker->m_config.trap.shoot.speed,
        speaker->m_config.trap.shoot.feedForward
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatAmp(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.amp.shoot.speed,
        speaker->m_config.amp.shoot.feedForward,
        speaker->m_config.amp.shoot.speed
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatSpeakerFar(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.speaker.far.shoot.speed,
        speaker->m_config.speaker.far.shoot.feedForward,
        speaker->m_config.speaker.far.shoot.speed
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatSpeakerNear(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.speaker.near.shoot.speed,
        speaker->m_config.speaker.near.shoot.feedForward,
        speaker->m_config.speaker.near.shoot.speed
    ).ToPtr();
}

frc2::CommandPtr robot2::cmd::PreheatTrap(
    SpeakerShooterSubsystem * speaker
) {
    return PreheatShooter(
        speaker,
        speaker->m_config.trap.shoot.speed,
        speaker->m_config.trap.shoot.feedForward,
        speaker->m_config.trap.shoot.speed
    ).ToPtr();
}
