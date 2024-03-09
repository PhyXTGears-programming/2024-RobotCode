#pragma once //makes it so that parent can only be used once

/*******************************
 a parent always has a child a parent in coding also has a
 child example of a parent: animal is the parent of dog
 this would also applie to coding so (line of code)
 is parent of (other line of code)
*******************************/

#include "external/cpptoml.h"
#include "Interface.h"

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angular_velocity.h>
#include <units/length.h>

#include <rev/CANSparkMax.h>

using rpm_t = units::revolutions_per_minute_t;

class SpeakerShooterSubsystem : public frc2::SubsystemBase { //SubsystemBase is the Parent of SpeakerShooterSubsystem
    public:
        SpeakerShooterSubsystem(std::shared_ptr<cpptoml::table> table);

        void Shoot();
        void ReverseShooter();
        void StopShooter();

        bool IsNoteDetected();
        bool IsSpeakerNear();

        rpm_t GetShooterSpeed();

        void SetShooterSpeed(rpm_t speed);

        units::meter_t GetSpeakerDistance();

    private:
    
        rev::CANSparkMax m_shootMotor1;
        rev::SparkPIDController m_shootPid1;
        rev::SparkRelativeEncoder m_shootEncoder1;

        rev::CANSparkMax m_shootMotor2;

        frc::DigitalInput m_noteSensor;

        // Config settings loaded from TOML.
        struct {
            rpm_t shootSpeed;
            rpm_t reverseSpeed;
        } m_config;
};
