#include "robots/2/Interface.h"
#include "robots/2/subsystems/shooter_tilt/ShooterTilt.h"

#include <cmath>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>

using namespace ::robot2;

robot2::ShooterTiltSubsystem::ShooterTiltSubsystem(
    std::shared_ptr<cpptoml::table> table
) :
    m_tiltMotorLeft(interface::tilt::k_tiltLeft, rev::CANSparkMax::MotorType::kBrushless),
    m_tiltMotorRight(interface::tilt::k_tiltRight, rev::CANSparkMax::MotorType::kBrushless),
    // Invert measurements so that tilt-up is more positive, and tilt-down is
    // more negative. Bias with +0.5 to move range from [-??, +??] to
    // [0.0, 1.0].
    m_tiltSensor(interface::tilt::k_tiltSensor, -1.0, 1.0)
{
    bool hasError = false;

    // Load configuration values from TOML.
    {
        cpptoml::option<double> limit = table->get_qualified_as<double>("topLimit");

        if (limit) {
            m_config.topLimit = *limit;
        } else {
            std::cerr << "Error: tilt cannot find toml tilt.topLimit" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> limit = table->get_qualified_as<double>("bottomLimit");

        if (limit) {
            m_config.bottomLimit = *limit;
        } else {
            std::cerr << "Error: tilt cannot find toml tilt.bottomLimit" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> setpoint = table->get_qualified_as<double>("setpoint.speaker");

        if (setpoint) {
            m_config.setpoint.speaker = *setpoint;
        } else {
            std::cerr << "Error: tilt cannot find toml tilt.setpoint.speaker" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> setpoint = table->get_qualified_as<double>("setpoint.stage");

        if (setpoint) {
            m_config.setpoint.stage = *setpoint;
        } else {
            std::cerr << "Error: tilt cannot find toml tilt.setpoint.stage" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("maxSpeed");

        if (speed) {
            m_config.maxSpeed = *speed;
        } else {
            std::cerr << "Error: tilt cannot find toml tilt.maxSpeed" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    m_tiltMotorLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_tiltMotorRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_tiltMotorLeft.SetInverted(true);
    m_tiltMotorRight.SetInverted(true);

    m_tiltMotorRight.Follow(m_tiltMotorLeft, false);
}

void robot2::ShooterTiltSubsystem::Periodic() {
    UpdatePosition();
}

void robot2::ShooterTiltSubsystem::SetTilt(double position, double speed) {
    double measure = m_tiltPosition.Get();

    double diff = position - measure;

    if (0.001 > std::abs(diff)) {
        SetTiltSpeed(0.0);  // Stop.
    } else {
        SetTiltSpeed(std::copysign(speed, diff));
    }
}

void robot2::ShooterTiltSubsystem::SetTiltSpeed(double speed) {
    // Prevent motor from crashing.
    // It's imperative that the limit setpoints are accurate, else this does nothing to
    // protect the robot.
    if ((speed > 0.0 && IsAboveTopLimit())
        || (speed < 0.0 && IsBelowBottomLimit()))
    {
        m_tiltMotorLeft.Set(0.0);
    } else {
        m_tiltMotorLeft.Set(std::clamp(speed, -1.0, 1.0) * m_config.maxSpeed);
    }
}

void robot2::ShooterTiltSubsystem::UpdatePosition() {
    m_tiltPosition.Update(m_tiltSensor.Get());
}

double robot2::ShooterTiltSubsystem::GetPosition() const {
    return m_tiltPosition.Get();
}

bool robot2::ShooterTiltSubsystem::IsAboveTopLimit() const {
    return GetPosition() > m_config.topLimit;
}

bool robot2::ShooterTiltSubsystem::IsBelowBottomLimit() const {
    return GetPosition() < m_config.bottomLimit;
}

void robot2::ShooterTiltSubsystem::GotoSpeakerPosition() {
    SetTilt(m_config.setpoint.speaker, m_config.maxSpeed);
}

void robot2::ShooterTiltSubsystem::GotoStagePosition() {
    SetTilt(m_config.setpoint.stage, m_config.maxSpeed);
}

double robot2::ShooterTiltSubsystem::GetSpeakerPosition() const {
    return m_config.setpoint.speaker;
}

double robot2::ShooterTiltSubsystem::GetStagePosition() const {
    return m_config.setpoint.stage;
}
