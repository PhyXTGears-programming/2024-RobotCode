#pragma once

#include "subsystems/drivetrain/Drivetrain.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

/**
 * Registry class to hold collection of subsystems dependencies for command
 * creation.
 */
struct SubsystemRegistry {
    Drivetrain * drivetrain;
    IntakeSubsystem * intake;
    SpeakerShooterSubsystem * speaker;
};
