#include "Interface.h"
#include "subsystems/gate/Gate.h"

#include <frc2/command/SubsystemBase.h>


GateSubsystem::GateSubsystem(std::shared_ptr<cpptoml::table> table)
:   m_servo(Interface::k_gateServo)
{
    {
        cpptoml::option<double> servo_time = table->get_qualified_as<double>("openMs");

        if (!servo_time) {
            throw "Error: gate cannot find toml gate.OpenMs";
        }

        m_config.openMs = units::microsecond_t(*servo_time);
    }

    {
        cpptoml::option<double> servo_time = table->get_qualified_as<double>("closeMs");

        if (!servo_time) {
            throw "Error: gate cannot find toml gate.closeMs";
        }

        m_config.closeMs = units::microsecond_t(*servo_time);
    }
}

void GateSubsystem::Open () {
    m_servo.SetPulseTime(m_config.openMs);
}

void GateSubsystem::Close () {
    m_servo.SetPulseTime(m_config.closeMs);
}