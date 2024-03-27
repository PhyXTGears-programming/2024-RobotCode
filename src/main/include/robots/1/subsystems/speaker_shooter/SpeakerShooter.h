#pragma once //makes it so that parent can only be used once

/*******************************
 a parent always has a child a parent in coding also has a
 child example of a parent: animal is the parent of dog
 this would also applie to coding so (line of code)
 is parent of (other line of code)
*******************************/

#include "external/cpptoml.h"
#include "robots/1/subsystems/speaker_shooter/DiagnosticDecl.h"

#include <frc/AsynchronousInterrupt.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angular_velocity.h>
#include <units/length.h>

#include <rev/CANSparkMax.h>

using rpm_t = units::revolutions_per_minute_t;

namespace robot1 {
    using namespace ::robot1;

    class SpeakerShooterSubsystem : public frc2::SubsystemBase { //SubsystemBase is the Parent of SpeakerShooterSubsystem
        public:
            SpeakerShooterSubsystem(std::shared_ptr<cpptoml::table> table);

            void Periodic() override;

            void Shoot();
            void SlowShoot();
            void ReverseShooter();
            void StopShooter();

            bool IsNoteDetected();
            bool IsSpeakerNear();

            rpm_t GetShooterSpeed();
            rpm_t GetFastSpeedThreshold();
            rpm_t GetSlowSpeedThreshold();

            void SetShooterSpeed(rpm_t speed, units::volt_t feedForward);

            units::meter_t GetSpeakerDistance();

        private:
        
            rev::CANSparkMax m_shootMotor1;
            rev::SparkPIDController m_shootPid1;
            rev::SparkRelativeEncoder m_shootEncoder1;

            rev::CANSparkMax m_shootMotor2;
            rev::SparkRelativeEncoder m_shootEncoder2;

            frc::DigitalInput m_noteSensor;
            frc::AsynchronousInterrupt m_noteInterrupt;

            bool m_isNoteDetected = false;
            bool m_isDetectFlagViewed = false;

            // Config settings loaded from TOML.
            struct {
                struct {
                    struct {
                        rpm_t speed;
                        units::volt_t feedForward;
                    } fast;
                    struct {
                        rpm_t speed;
                        units::volt_t feedForward;
                    } slow;
                } shoot;
                rpm_t reverseSpeed;
                units::meter_t  distanceThreshold;
                units::volt_t   feedForwardSlow;
            } m_config;

        friend class diagnostic::TestSpeaker;
    };

}
