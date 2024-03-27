#pragma once

#include "external/cpptoml.h"
#include "NoSparkMax.h"

#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

namespace robot1 {

    class AmpShooterSubsystem : public frc2::SubsystemBase {
        public:
            AmpShooterSubsystem(std::shared_ptr<cpptoml::table> table);

            void Lift();
            void Drop();
            void StopLift();

            bool IsLiftAtLimit();

            void Shoot();
            void StopShoot();

            void Extend();
            void Retract();
            bool IsExtended();


            bool IsNoteDetected();

        private:
            NoSparkMax m_liftMotor;
            NoSparkMax m_shootMotor;

            frc::Servo m_servo;

            frc::DigitalInput m_noteSensor;

            struct {
                double liftCurrentThreshold = 0.0;
                struct {
                    units::microsecond_t extendPulseTime = 0.0_us;
                    units::microsecond_t retractPulseTime = 0.0_us;
                } servo;
            } m_config;
    };

}
