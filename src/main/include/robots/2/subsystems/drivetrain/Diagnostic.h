#pragma once

#include "robots/2/subsystems/drivetrain/Drivetrain.h"

#include <frc/XboxController.h>

namespace robot2::diagnostic {
    class TestDrivetrain {
        public:
            TestDrivetrain(Drivetrain * drivetrain, frc::XboxController * controller);

            void Test01MeasureTurnConversionFactor();

            void Test02MeasureDriveConversionFactor();

            void Test03MeasureTurnAlignment();

            void Test04TuneTurnPid();

            void Test05TuneDrivePid();

            void Test06MeasureDriveConversionFactor();

            Drivetrain * m_drivetrain = nullptr;
            frc::XboxController * m_controller = nullptr;
    };
}
