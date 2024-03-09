#include "Interface.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <iostream>

#include <frc2/command/SubsystemBase.h>

SpeakerShooterSubsystem::SpeakerShooterSubsystem(std::shared_ptr<cpptoml::table> table)
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
    m_noteSensor(interface::speaker::k_noteSensor)
{
    bool hasError = false;

    // Load configuration values from TOML.
    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("shootSpeed");

        if (speed) {
            m_config.shootSpeed = rpm_t(*speed);
        } else {
            std::cerr << "Error: arm shooter cannot find toml property armShooter.shootSpeed" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("reverseSpeed");

        if (speed) {
            m_config.reverseSpeed = rpm_t(*speed);
        } else {
            std::cerr << "Error: arm shooter cannot find toml property armShooter.reverseSpeed" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    m_shootPid1.SetFeedbackDevice(m_shootEncoder1);

    // Shoot motor 2 shall follow motor 1 in reverse direction.
    m_shootMotor2.Follow(m_shootMotor1, true);

}

void SpeakerShooterSubsystem::Shoot() {
    SetShooterSpeed(m_config.shootSpeed);
}

void SpeakerShooterSubsystem::ReverseShooter() {
    SetShooterSpeed(m_config.reverseSpeed);
}

void SpeakerShooterSubsystem::StopShooter() {
    SetShooterSpeed(0_rpm);
}

bool SpeakerShooterSubsystem::IsNoteDetected() {
    return m_noteSensor.Get();
}

bool SpeakerShooterSubsystem::IsSpeakerNear() {}
