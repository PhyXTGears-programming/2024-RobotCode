#include "robots/1/Interface.h"
#include "rev/CANSparkMax.h"
#include "robots/1/subsystems/amp_shooter/AmpShooter.h"

#include <iostream>

#include <frc2/command/SubsystemBase.h>

using namespace ::robot1;

robot1::AmpShooterSubsystem::AmpShooterSubsystem(
    std::shared_ptr<cpptoml::table> table
) :
    m_liftMotor(interface::amp::k_liftMotor, rev::CANSparkMax::MotorType::kBrushless),
    m_shootMotor(interface::amp::k_shootMotor, rev::CANSparkMax::MotorType::kBrushless),
    m_servo(interface::amp::k_tiltServo),
    m_noteSensor(interface::amp::k_noteSensor)
{
    bool hasError = false;

    {
        cpptoml::option<double> liftCurrentThreshold = table->get_qualified_as<double>("liftCurrentThreshold");
        if (liftCurrentThreshold) {
            m_config.liftCurrentThreshold = *liftCurrentThreshold;
        } else {
            std::cerr << "Error: ampShooter cannot find toml property liftCurrentThreshold" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> servoExtendPulseMicro = table->get_qualified_as<double>("servo.extendPulseMicro");
        if (servoExtendPulseMicro) {
            m_config.servo.extendPulseTime = units::microsecond_t(*servoExtendPulseMicro);
        } else {
            std::cerr << "Error: ampShooter cannot find toml property servo.extendPulseTime" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> servoRetractPulseMicro = table->get_qualified_as<double>("servo.retractPulseMicro");
        if (servoRetractPulseMicro) {
            m_config.servo.retractPulseTime = units::microsecond_t(*servoRetractPulseMicro);
        } else {
            std::cerr << "Error: ampShooter cannot find toml property servo.retractPulseTime" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    m_liftMotor.SetInverted(false);
    m_liftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_liftMotor.SetSmartCurrentLimit(10);
    m_liftMotor.EnableVoltageCompensation(12.0);

    m_shootMotor.SetInverted(false);
    m_shootMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_shootMotor.SetSmartCurrentLimit(10);
    m_shootMotor.EnableVoltageCompensation(12.0);
}

void robot1::AmpShooterSubsystem::Lift() {
    m_liftMotor.Set(0.2);
}

void robot1::AmpShooterSubsystem::Drop() {
    m_liftMotor.Set(-0.1);
}

void robot1::AmpShooterSubsystem::StopLift() {
    m_liftMotor.StopMotor();
}

bool robot1::AmpShooterSubsystem::IsLiftAtLimit() {
    return m_config.liftCurrentThreshold <= m_liftMotor.GetOutputCurrent();
}

void robot1::AmpShooterSubsystem::Shoot() {
    m_shootMotor.Set(0.0);
}

void robot1::AmpShooterSubsystem::StopShoot() {
    m_shootMotor.StopMotor();
}

void robot1::AmpShooterSubsystem::Extend() {
    m_servo.SetPulseTime(m_config.servo.extendPulseTime);
}

void robot1::AmpShooterSubsystem::Retract() {
    m_servo.SetPulseTime(m_config.servo.retractPulseTime);
}

bool robot1::AmpShooterSubsystem::IsNoteDetected() {
    return !m_noteSensor.Get();
}
