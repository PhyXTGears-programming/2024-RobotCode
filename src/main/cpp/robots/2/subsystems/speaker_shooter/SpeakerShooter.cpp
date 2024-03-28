#include "robots/2/commands/PreheatShooter.h"
#include "robots/2/commands/Shoot.h"
#include "robots/2/Interface.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

using namespace ::robot2;

robot2::SpeakerShooterSubsystem::SpeakerShooterSubsystem(std::shared_ptr<cpptoml::table> table)
:   m_shootMotor1(
        interface::speaker::k_motor1,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    ),
    m_shootPid1(m_shootMotor1.GetPIDController()),
    m_shootEncoder1(m_shootMotor1.GetEncoder(
        rev::SparkRelativeEncoder::Type::kHallSensor
    )),
    m_shootMotor2(
        interface::speaker::k_motor2,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    ),
    m_shootEncoder2(m_shootMotor2.GetEncoder(
        rev::SparkRelativeEncoder::Type::kHallSensor
    )),
    m_noteSensor(interface::speaker::k_noteSensor),
    m_noteInterrupt(
        m_noteSensor,
        [this] (bool, bool) {
            m_isNoteDetected = true;
            m_isDetectFlagViewed = false;
        }
    )
{
    bool hasError = false;

    // Load configuration values from TOML.
    {
        // amp.shoot.speed
        cpptoml::option<double> speed = table->get_qualified_as<double>("amp.shoot.speed");

        if (speed) {
            m_config.amp.shoot.speed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.amp.shoot.speed" << std::endl;
            hasError = true;
        }
    }

    {
        // amp.shoot.feedForward
        cpptoml::option<double> feedForward = table->get_qualified_as<double>("amp.shoot.feedForward");

        if (feedForward) {
            m_config.amp.shoot.feedForward = units::volt_t(*feedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.amp.shoot.feedForward" << std::endl;
            hasError = true;
        }
    }

    {
        // amp.tilt.leftMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("amp.tilt.leftMicros");

        if (pulseTime) {
            m_config.amp.tilt.leftMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.amp.tilt.leftMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // amp.tilt.rightMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("amp.tilt.rightMicros");

        if (pulseTime) {
            m_config.amp.tilt.rightMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.amp.tilt.rightMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.far.shoot.speed
        cpptoml::option<double> speed = table->get_qualified_as<double>("speaker.far.shoot.speed");

        if (speed) {
            m_config.speaker.far.shoot.speed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.far.shoot.speed" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.far.shoot.feedForward
        cpptoml::option<double> feedForward = table->get_qualified_as<double>("speaker.far.shoot.feedForward");

        if (feedForward) {
            m_config.speaker.far.shoot.feedForward = units::volt_t(*feedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.far.shoot.feedForward" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.far.tilt.leftMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("speaker.far.tilt.leftMicros");

        if (pulseTime) {
            m_config.speaker.far.tilt.leftMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.far.tilt.leftMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.far.tilt.rightMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("speaker.far.tilt.rightMicros");

        if (pulseTime) {
            m_config.speaker.far.tilt.rightMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.far.tilt.rightMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.near.shoot.speed
        cpptoml::option<double> speed = table->get_qualified_as<double>("speaker.near.shoot.speed");

        if (speed) {
            m_config.speaker.near.shoot.speed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.near.shoot.speed" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.near.shoot.feedForward
        cpptoml::option<double> feedForward = table->get_qualified_as<double>("speaker.near.shoot.feedForward");

        if (feedForward) {
            m_config.speaker.near.shoot.feedForward = units::volt_t(*feedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.near.shoot.feedForward" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.near.tilt.leftMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("speaker.near.tilt.leftMicros");

        if (pulseTime) {
            m_config.speaker.near.tilt.leftMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.near.tilt.leftMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // speaker.near.tilt.rightMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("speaker.near.tilt.rightMicros");

        if (pulseTime) {
            m_config.speaker.near.tilt.rightMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.speaker.near.tilt.rightMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // trap.shoot.speed
        cpptoml::option<double> speed = table->get_qualified_as<double>("trap.shoot.speed");

        if (speed) {
            m_config.trap.shoot.speed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.trap.shoot.speed" << std::endl;
            hasError = true;
        }
    }

    {
        // trap.shoot.feedForward
        cpptoml::option<double> feedForward = table->get_qualified_as<double>("trap.shoot.feedForward");

        if (feedForward) {
            m_config.trap.shoot.feedForward = units::volt_t(*feedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.trap.shoot.feedForward" << std::endl;
            hasError = true;
        }
    }

    {
        // trap.tilt.leftMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("trap.tilt.leftMicros");

        if (pulseTime) {
            m_config.trap.tilt.leftMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.trap.tilt.leftMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // trap.tilt.rightMicros
        cpptoml::option<double> pulseTime = table->get_qualified_as<double>("trap.tilt.rightMicros");

        if (pulseTime) {
            m_config.trap.tilt.rightMicros = units::microsecond_t(*pulseTime);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.trap.tilt.rightMicros" << std::endl;
            hasError = true;
        }
    }

    {
        // reverseSpeed
        cpptoml::option<double> speed = table->get_qualified_as<double>("reverseSpeed");

        if (speed) {
            m_config.reverseSpeed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.reverseSpeed" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> distanceThreshold = table->get_qualified_as<double>("distanceThreshold");

        if (distanceThreshold) {
            m_config.distanceThreshold = units::meter_t(*distanceThreshold);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.distanceThreshold" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    m_shootPid1.SetFeedbackDevice(m_shootEncoder1);

    // Shoot motor 2 shall follow motor 1 in reverse direction.
    m_shootMotor2.Follow(m_shootMotor1, true);

    m_noteInterrupt.SetInterruptEdges(false, true);
    m_noteInterrupt.Enable();
}

void robot2::SpeakerShooterSubsystem::Periodic() {
    if (m_isDetectFlagViewed) {
        m_isNoteDetected = false;
        m_isDetectFlagViewed = false;
    }

    frc::SmartDashboard::PutBoolean("Speaker Note Detected", IsNoteDetected());
    frc::SmartDashboard::PutNumber("Speaker Shoot rpm", GetShooterSpeed().value());
}

void robot2::SpeakerShooterSubsystem::ReverseShooter() {
    SetShooterSpeed(
        m_config.reverseSpeed,
        m_config.amp.shoot.feedForward
    );
}

void robot2::SpeakerShooterSubsystem::StopShooter() {
    m_shootMotor1.StopMotor();
}

bool robot2::SpeakerShooterSubsystem::IsNoteDetected() {
    m_isDetectFlagViewed = true;
    return m_isNoteDetected || !m_noteSensor.Get();
}

units::meter_t robot2::SpeakerShooterSubsystem::GetSpeakerDistance(){
    return units::meter_t(-1);
}

bool robot2::SpeakerShooterSubsystem::IsSpeakerNear() {
    return std::abs((m_config.distanceThreshold - GetSpeakerDistance()).value()) < 0.1;
}

rpm_t robot2::SpeakerShooterSubsystem::GetShooterSpeed(){
    return rpm_t(m_shootEncoder1.GetVelocity());
}

rpm_t robot2::SpeakerShooterSubsystem::GetAmpSpeedThreshold() {
    return m_config.amp.shoot.speed;
}

rpm_t robot2::SpeakerShooterSubsystem::GetSpeakerFarSpeedThreshold() {
    return m_config.speaker.far.shoot.speed;
}

rpm_t robot2::SpeakerShooterSubsystem::GetSpeakerNearSpeedThreshold() {
    return m_config.speaker.near.shoot.speed;
}

rpm_t robot2::SpeakerShooterSubsystem::GetTrapSpeedThreshold() {
    return m_config.trap.shoot.speed;
}

void robot2::SpeakerShooterSubsystem::SetShooterSpeed(
    rpm_t speed,
    units::volt_t feedForward
) {
    m_shootPid1.SetReference(
        speed.value(),
        rev::ControlType::kVelocity,
        0,
        std::copysign(feedForward.value(), speed.value())
    );
}
