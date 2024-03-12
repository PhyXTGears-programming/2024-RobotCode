#pragma once

#include "subsystems/drivetrain/Drivetrain.h"

#include <frc/XboxController.h>

namespace diagnostic {
    class TestDrivetrain {
        public:
            TestDrivetrain(Drivetrain * drivetrain, frc::XboxController * controller);

            void Test01MeasureTurnConversionFactor();

            void Test02MeasureDriveConversionFactor();

            void Test03MeasureTurnAlignment();

            void Test04TuneTurnPid();

            void Test05TuneDrivePid();

            Drivetrain * m_drivetrain = nullptr;
            frc::XboxController * m_controller = nullptr;
    };
}
