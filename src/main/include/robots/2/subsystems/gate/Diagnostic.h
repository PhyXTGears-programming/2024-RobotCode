#pragma once

#include "robots/2/subsystems/gate/Gate.h"

#include <frc/XboxController.h>

namespace robot2::diagnostic {
    using namespace ::robot2;

    class TestGate {
        public:
            TestGate(GateSubsystem * gate, frc::XboxController * controller);

            void Test01TuneServo();

            GateSubsystem * m_gate = nullptr;
            frc::XboxController * m_controller = nullptr;
    };
}
