#include "Interface.h"
#include "subsystems/gate/Gate.h"

#include <frc2/command/SubsystemBase.h>


GateSubsystem::GateSubsystem(std::shared_ptr<cpptoml::table> table)
:   m_servo(Interface::k_gateServo)
{
    {
        cpptoml::option<double> servo_time = table->get_qualified_as<double>("openMicros");

        if (!servo_time) {
            throw "Error: gate cannot find toml gate.OpenMicros";
        }

        m_config.openMicros = units::microsecond_t(*servo_time);
    }

    {
        cpptoml::option<double> servo_time = table->get_qualified_as<double>("closeMicros");

        if (!servo_time) {
            throw "Error: gate cannot find toml gate.closeMicros";
        }

        m_config.closeMicros = units::microsecond_t(*servo_time);
    }
}

void GateSubsystem::Open () {
    m_servo.SetPulseTime(m_config.openMicros);
}

void GateSubsystem::Close () {
    m_servo.SetPulseTime(m_config.closeMicros);
}
