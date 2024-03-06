#pragma once

#include "external/cpptoml.h"

#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

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
        rev::CANSparkMax m_liftMotor;
        rev::CANSparkMax m_shootMotor;

        frc::Servo m_servo;

        frc::DigitalInput m_noteSensor;

        struct {
            double liftCurrentThreshold = 0.0;
            struct {
                double extendAngleDeg = 0.0;
                double retractAngleDeg = 0.0;
            } servo;
        } m_config;
};
