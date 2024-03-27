#pragma once

#include "robots/1/subsystems/gate/Gate.h"

#include <frc/XboxController.h>

namespace robot1::diagnostic {
    using namespace ::robot1;

    class TestGate {
        public:
            TestGate(GateSubsystem * gate, frc::XboxController * controller);

            void Test01TuneServo();

            GateSubsystem * m_gate = nullptr;
            frc::XboxController * m_controller = nullptr;
    };
}
