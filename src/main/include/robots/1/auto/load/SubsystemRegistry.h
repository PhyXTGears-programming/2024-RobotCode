#pragma once

#include "robots/1/subsystems/drivetrain/Drivetrain.h"
#include "robots/1/subsystems/intake/Intake.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

namespace robot1 {
    /**
    * Registry class to hold collection of subsystems dependencies for command
    * creation.
    */
    struct SubsystemRegistry {
        Drivetrain * drivetrain;
        IntakeSubsystem * intake;
        SpeakerShooterSubsystem * speaker;
    };
}
