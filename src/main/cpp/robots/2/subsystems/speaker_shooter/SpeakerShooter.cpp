#include "robots/2/Interface.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

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
        // shoot.fast.speed
        cpptoml::option<double> speed = table->get_qualified_as<double>("shoot.fast.speed");

        if (speed) {
            m_config.shoot.fast.speed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.shoot.fast.speed" << std::endl;
            hasError = true;
        }
    }

    {
        // shoot.fast.feedForward
        cpptoml::option<double> feedForward = table->get_qualified_as<double>("shoot.fast.feedForward");

        if (feedForward) {
            m_config.shoot.fast.feedForward = units::volt_t(*feedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.shoot.fast.feedForward" << std::endl;
            hasError = true;
        }
    }

    {
        // shoot.slow.speed
        cpptoml::option<double> speed = table->get_qualified_as<double>("shoot.slow.speed");

        if (speed) {
            m_config.shoot.slow.speed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.shoot.slow.speed" << std::endl;
            hasError = true;
        }
    }

    {
        // shoot.slow.feedForward
        cpptoml::option<double> feedForward = table->get_qualified_as<double>("shoot.slow.feedForward");

        if (feedForward) {
            m_config.shoot.slow.feedForward = units::volt_t(*feedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.shoot.slow.feedForward" << std::endl;
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

void robot2::SpeakerShooterSubsystem::Shoot() {
    SetShooterSpeed(
        m_config.shoot.fast.speed,
        m_config.shoot.fast.feedForward
    );
}

void robot2::SpeakerShooterSubsystem::SlowShoot() {
    SetShooterSpeed(
        m_config.shoot.slow.speed,
        m_config.shoot.slow.feedForward
    );
}

void robot2::SpeakerShooterSubsystem::ReverseShooter() {
    SetShooterSpeed(
        m_config.reverseSpeed,
        m_config.shoot.slow.feedForward
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

rpm_t robot2::SpeakerShooterSubsystem::GetFastSpeedThreshold() {
    return m_config.shoot.fast.speed;
}

rpm_t robot2::SpeakerShooterSubsystem::GetSlowSpeedThreshold() {
    return m_config.shoot.slow.speed;
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
