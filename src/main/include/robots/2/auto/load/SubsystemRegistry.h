#pragma once

#include "robots/2/subsystems/drivetrain/Drivetrain.h"
#include "robots/2/subsystems/intake/Intake.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

namespace robot2 {
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
