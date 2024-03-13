#pragma once

#include "subsystems/gate/Gate.h"

#include <frc/XboxController.h>

namespace diagnostic {
    class TestGate {
        public:
            TestGate(GateSubsystem * gate, frc::XboxController * controller);

            void Test01TuneServo();

            GateSubsystem * m_gate = nullptr;
            frc::XboxController * m_controller = nullptr;
    };
}
