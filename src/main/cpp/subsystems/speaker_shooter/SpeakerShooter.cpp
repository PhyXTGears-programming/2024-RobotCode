#include "Interface.h"
#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <cmath>
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
    m_shootEncoder2(m_shootMotor2.GetEncoder(
        rev::SparkRelativeEncoder::Type::kHallSensor
    )),
    m_noteSensor(interface::speaker::k_noteSensor)
{
    bool hasError = false;

    // Load configuration values from TOML.
    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("shootSpeed");

        if (speed) {
            m_config.shootSpeed = rpm_t(*speed);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.shootSpeed" << std::endl;
            hasError = true;
        }
    }

    {
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
        cpptoml::option<double> arbFeedForward = table->get_qualified_as<double>("arbFeedForward");

        if (arbFeedForward) {
            m_config.arbFeedForward = units::volt_t(*arbFeedForward);
        } else {
            std::cerr << "Error: speaker shooter cannot find toml property speaker.arbFeedForward" << std::endl;
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
    m_shootMotor1.StopMotor();
}

bool SpeakerShooterSubsystem::IsNoteDetected() {
    return m_noteSensor.Get();
}

units::meter_t SpeakerShooterSubsystem::GetSpeakerDistance(){
    return units::meter_t(-1);
}

bool SpeakerShooterSubsystem::IsSpeakerNear() {
    return std::abs((m_config.distanceThreshold - GetSpeakerDistance()).value()) < 0.1;
}

rpm_t SpeakerShooterSubsystem::GetShooterSpeed(){
    return rpm_t(m_shootEncoder1.GetVelocity());
}

void SpeakerShooterSubsystem::SetShooterSpeed(rpm_t speed){
    m_shootPid1.SetReference(
        speed.value(),
        rev::ControlType::kVelocity,
        0,
        std::copysign(m_config.arbFeedForward.value(), speed.value())
    );
}
