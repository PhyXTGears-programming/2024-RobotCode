#include "robots/1/Interface.h"
#include "robots/1/subsystems/gate/Gate.h"

#include <iostream>

#include <frc2/command/SubsystemBase.h>


robot1::GateSubsystem::GateSubsystem(std::shared_ptr<cpptoml::table> table)
:   m_servo(interface::gate::k_servo)
{
    bool hasError = false;

    {
        cpptoml::option<double> servo_time = table->get_qualified_as<double>("openMicros");

        if (servo_time) {
            m_config.openMicros = units::microsecond_t(*servo_time);
        } else {
            std::cerr << "Error: gate cannot find toml gate.OpenMicros" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> servo_time = table->get_qualified_as<double>("closeMicros");

        if (servo_time) {
            m_config.closeMicros = units::microsecond_t(*servo_time);
        } else {
            std::cerr << "Error: gate cannot find toml gate.closeMicros" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }
}

void robot1::GateSubsystem::Open () {
    m_servo.SetPulseTime(m_config.openMicros);
}

void robot1::GateSubsystem::Close () {
    m_servo.SetPulseTime(m_config.closeMicros);
}
