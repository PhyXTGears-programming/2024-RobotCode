#pragma once

#include "subsystems/intake/Intake.h"

#include <frc/XboxController.h>

namespace diagnostic {
    class TestIntake {
        public:
            TestIntake(IntakeSubsystem * intake, frc::XboxController * controller);

            void Test01TuneIntake();

            IntakeSubsystem * m_intake = nullptr;
            frc::XboxController * m_controller = nullptr;
    };
}
