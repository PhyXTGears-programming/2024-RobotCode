#pragma once

#include "external/cpptoml.h"

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>


class GateSubsystem : public frc2::SubsystemBase {
    public:
        GateSubsystem(std::shared_ptr<cpptoml::table> table);

        void Open();
        void Close();

    private:
        frc::Servo m_servo;

        struct {
            units::microsecond_t openMs;
            units::microsecond_t closeMs;

        } m_config;
};
