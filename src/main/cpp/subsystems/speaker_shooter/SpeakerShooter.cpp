#include "subsystems/speaker_shooter/SpeakerShooter.h"

#include <cmath>

#include <frc2/command/SubsystemBase.h>

SpeakerShooterSubsystem::SpeakerShooterSubsystem(std::shared_ptr<cpptoml::table> table)
    : m_shootMotor1(
        Interface::k_speakerShooterMotor1,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    ),
    m_shootPid1(m_shootMotor1.GetPIDController()),
    m_shootEncoder1(m_shootMotor1.GetEncoder(
        rev::SparkRelativeEncoder::Type::kHallSensor
    )),
    m_shootMotor2(
        Interface::k_speakerShooterMotor2,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    ),
    m_noteSensor(Interface::k_speakerShooterNoteSensor)
{
    // Load configuration values from TOML.
    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("shootSpeed");

        if (!speed) {
            throw "Error: arm shooter cannot find toml property armShooter.shootSpeed";
        }

        m_config.shootSpeed = rpm_t(*speed);
    }

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("reverseSpeed");

        if (!speed) {
            throw "Error: arm shooter cannot find toml property armShooter.reverseSpeed";
        }

        m_config.reverseSpeed = rpm_t(*speed);
    }

    {
        cpptoml::option<double> distanceThreshold = table->get_qualified_as<double>("distanceThreshold");

        if (!distanceThreshold) {
            throw "Error: arm shooter cannot find toml property armShooter.distanceThreshold";
        }

        m_config.distanceThreshold = units::meter_t(*distanceThreshold);
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

units::meter_t SpeakerShooterSubsystem::GetSpeakerDistance(){
    return units::meter_t(-1);
}

bool SpeakerShooterSubsystem::IsSpeakerNear() {
    return std::abs((m_config.distanceThreshold - GetSpeakerDistance()).value()) < 0.1;
}

rpm_t SpeakerShooterSubsystem::GetShooterSpeed(){
// idk how to that
}

void SpeakerShooterSubsystem::SetShooterSpeed(rpm_t speed){
// if I could see my intake sub then I could
}
