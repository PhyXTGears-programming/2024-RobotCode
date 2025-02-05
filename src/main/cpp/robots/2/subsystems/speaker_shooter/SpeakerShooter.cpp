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
        rev::CANSparkMax::MotorType::kBrushless
    ),
    m_shootPid1(m_shootMotor1.GetPIDController()),
    m_shootEncoder1(m_shootMotor1.GetEncoder(
        rev::SparkRelativeEncoder::Type::kHallSensor
    )),
    m_shootMotor2(
        interface::speaker::k_motor2,
        rev::CANSparkMax::MotorType::kBrushless
    ),
    m_shootEncoder2(m_shootMotor2.GetEncoder(
        rev::SparkRelativeEncoder::Type::kHallSensor
    )),
    m_feedMotor(
        interface::speaker::k_feedMotor,
        rev::CANSparkMax::MotorType::kBrushless
    ),
    m_noteSensorBottom(interface::speaker::k_noteSensorBottom),
    m_noteSensorTop(interface::speaker::k_noteSensorTop),
    m_noteInterrupt(
        m_noteSensorBottom,
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

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("feed.intakeSpeed");

        if (speed) {
            m_config.feed.intakeSpeed = *speed;
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.feed.intakeSpeed" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("feed.reverseSpeed");

        if (speed) {
            m_config.feed.reverseSpeed = *speed;
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.feed.reverseSpeed" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("feed.shootSpeed");

        if (speed) {
            m_config.feed.shootSpeed = *speed;
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.feed.shootSpeed" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    m_shootPid1.SetFeedbackDevice(m_shootEncoder1);

    m_shootMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_shootMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Shoot motor 2 shall follow motor 1 in reverse direction.
    m_shootMotor2.Follow(m_shootMotor1, true);

    m_feedMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_feedMotor.SetInverted(true);

    m_noteInterrupt.SetInterruptEdges(false, true);
    m_noteInterrupt.Enable();
}

void robot2::SpeakerShooterSubsystem::Periodic() {
    if (m_isDetectFlagViewed) {
        m_isNoteDetected = false;
        m_isDetectFlagViewed = false;
    }

    frc::SmartDashboard::PutBoolean("Speaker Note Detected", IsNoteDetectedBottom());
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

bool robot2::SpeakerShooterSubsystem::IsNoteDetectedBottom() {
    m_isDetectFlagViewed = true;
    return m_isNoteDetected || !m_noteSensorBottom.Get();
}

bool robot2::SpeakerShooterSubsystem::IsNoteDetectedTop() {
    return !m_noteSensorTop.Get();
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

void robot2::SpeakerShooterSubsystem::IntakeNote() {
    m_feedMotor.Set(m_config.feed.intakeSpeed);
}

void robot2::SpeakerShooterSubsystem::FeedNote() {
    m_feedMotor.Set(m_config.feed.shootSpeed);
}

void robot2::SpeakerShooterSubsystem::StopFeed() {
    m_feedMotor.Set(0.0);
}

void robot2::SpeakerShooterSubsystem::ReverseFeed() {
    m_feedMotor.Set(m_config.feed.reverseSpeed);
}
