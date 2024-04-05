#pragma once //makes it so that parent can only be used once

/*******************************
 a parent always has a child a parent in coding also has a
 child example of a parent: animal is the parent of dog
 this would also applie to coding so (line of code)
 is parent of (other line of code)
*******************************/

#include "external/cpptoml.h"

#include "robots/2/commands/Commands.h"
#include "robots/2/subsystems/speaker_shooter/DiagnosticDecl.h"

#include <frc/AsynchronousInterrupt.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angular_velocity.h>
#include <units/length.h>

#include <rev/CANSparkMax.h>

using rpm_t = units::revolutions_per_minute_t;

namespace robot2 {
    using namespace ::robot2;

    class SpeakerShooterSubsystem : public frc2::SubsystemBase { //SubsystemBase is the Parent of SpeakerShooterSubsystem
        public:
            SpeakerShooterSubsystem(std::shared_ptr<cpptoml::table> table);

            void Periodic() override;

            void ReverseShooter();
            void StopShooter();

            bool IsNoteDetected();
            bool IsSpeakerNear();

            void TiltSpeakerFar();
            void TiltSpeakerNear();
            void TiltAmp();
            void TiltTrap();

            rpm_t GetShooterSpeed();

            rpm_t GetAmpSpeedThreshold();
            rpm_t GetSpeakerFarSpeedThreshold();
            rpm_t GetSpeakerNearSpeedThreshold();
            rpm_t GetTrapSpeedThreshold();

            void SetShooterSpeed(rpm_t speed, units::volt_t feedForward);

            units::meter_t GetSpeakerDistance();

            void IntakeNote();
            void FeedNote();
            void StopFeed();
            void ReverseFeed();

            /**
             * @param ratio Tilt of the shooter: 0.0 is lowest tilt, 1.0 is highest tilt.
             */
            void SetTilt(double ratio);

        private:

            rev::CANSparkMax m_shootMotor1;
            rev::SparkPIDController m_shootPid1;
            rev::SparkRelativeEncoder m_shootEncoder1;

            rev::CANSparkMax m_shootMotor2;
            rev::SparkRelativeEncoder m_shootEncoder2;

            rev::CANSparkMax m_feedMotor;

            frc::DigitalInput m_noteSensor;
            frc::AsynchronousInterrupt m_noteInterrupt;

            bool m_isNoteDetected = false;
            bool m_isDetectFlagViewed = false;

            frc::Servo m_tiltLeft;
            frc::Servo m_tiltRight;

            // Config settings loaded from TOML.
            struct {
                struct {
                    struct {
                        rpm_t speed;
                        units::volt_t feedForward;
                    } shoot;
                    double tilt;
                } amp;

                struct {
                    struct {
                        struct {
                            rpm_t speed;
                            units::volt_t feedForward;
                        } shoot;
                        double tilt;
                    } far;

                    struct {
                        struct {
                            rpm_t speed;
                            units::volt_t feedForward;
                        } shoot;
                        double tilt;
                    } near;
                } speaker;

                struct {
                    struct {
                        rpm_t speed;
                        units::volt_t feedForward;
                    } shoot;
                    double tilt;
                } trap;

                struct {
                    double intakeSpeed;
                    double reverseSpeed;
                    double shootSpeed;
                } feed;

                rpm_t reverseSpeed;
                units::meter_t  distanceThreshold;
            } m_config;

        friend class diagnostic::TestSpeaker;

        friend frc2::CommandPtr cmd::ShootAmp(SpeakerShooterSubsystem *);
        friend frc2::CommandPtr cmd::ShootSpeakerFar(SpeakerShooterSubsystem *);
        friend frc2::CommandPtr cmd::ShootSpeakerNear(SpeakerShooterSubsystem *);
        friend frc2::CommandPtr cmd::ShootTrap(SpeakerShooterSubsystem *);

        friend frc2::CommandPtr cmd::PreheatAmp(SpeakerShooterSubsystem *);
        friend frc2::CommandPtr cmd::PreheatSpeakerFar(SpeakerShooterSubsystem *);
        friend frc2::CommandPtr cmd::PreheatSpeakerNear(SpeakerShooterSubsystem *);
        friend frc2::CommandPtr cmd::PreheatTrap(SpeakerShooterSubsystem *);
    };

}
