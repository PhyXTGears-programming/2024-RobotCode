#pragma once

#include <frc2/command/CommandPtr.h>

// Forward declaration
namespace robot2 {
    class IntakeSubsystem;
    class SpeakerShooterSubsystem;
};

namespace robot2::cmd {

    using namespace ::robot2;

    frc2::CommandPtr Intake(IntakeSubsystem *, SpeakerShooterSubsystem *);

    frc2::CommandPtr ShootAmp(SpeakerShooterSubsystem *);
    frc2::CommandPtr ShootSpeakerFar(SpeakerShooterSubsystem *);
    frc2::CommandPtr ShootSpeakerNear(SpeakerShooterSubsystem *);
    frc2::CommandPtr ShootTrap(SpeakerShooterSubsystem *);

    frc2::CommandPtr PreheatAmp(SpeakerShooterSubsystem *);
    frc2::CommandPtr PreheatSpeakerFar(SpeakerShooterSubsystem *);
    frc2::CommandPtr PreheatSpeakerNear(SpeakerShooterSubsystem *);
    frc2::CommandPtr PreheatTrap(SpeakerShooterSubsystem *);

}
