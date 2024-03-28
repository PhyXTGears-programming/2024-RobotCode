#pragma once

#include "external/cpptoml.h"
#include "robots/2/subsystems/gate/DiagnosticDecl.h"

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

namespace robot2 {

    class GateSubsystem : public frc2::SubsystemBase {
        public:
            GateSubsystem(std::shared_ptr<cpptoml::table> table);

            void Open();
            void Close();

        private:
            frc::Servo m_servo;

            struct {
                units::microsecond_t openMicros;
                units::microsecond_t closeMicros;

            } m_config;

        friend class robot2::diagnostic::TestGate;
    };

}