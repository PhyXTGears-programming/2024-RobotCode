#include "Interface.h"
#include "rev/CANSparkMax.h"
#include "subsystems/amp_shooter/AmpShooter.h"

#include <frc2/command/SubsystemBase.h>

AmpShooterSubsystem::AmpShooterSubsystem(
    std::shared_ptr<cpptoml::table> table
) :
    m_liftMotor(Interface::k_ampShooterLiftMotor, rev::CANSparkMax::MotorType::kBrushless),
    m_shootMotor(Interface::k_ampShooterShootMotor, rev::CANSparkMax::MotorType::kBrushless),
    m_servo(Interface::k_ampShooterServo),
    m_noteSensor(Interface::k_ampShooterNoteSensor)
{
    cpptoml::option<double> liftCurrentThreshold = table->get_qualified_as<double>("liftCurrentThreshold");
    if (!liftCurrentThreshold) {
        throw "Error: ampShooter cannot find toml property liftCurrentThreshold";
    }
    m_config.liftCurrentThreshold = *liftCurrentThreshold;

    cpptoml::option<double> servoExtendPulseMicro = table->get_qualified_as<double>("servo.extendPulseMicro");
    if (!servoExtendPulseMicro) {
        throw "Error: ampShooter cannot find toml property servo.extendPulseTime";
    }
    m_config.servo.extendPulseTime = units::microsecond_t(*servoExtendPulseMicro);

    cpptoml::option<double> servoRetractPulseMicro = table->get_qualified_as<double>("servo.retractPulseMicro");
    if (!servoRetractPulseMicro) {
        throw "Error: ampShooter cannot find toml property servo.retractPulseTime";
    }
    m_config.servo.retractPulseTime = units::microsecond_t(*servoRetractPulseMicro);

    m_liftMotor.SetInverted(false);
    m_liftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_liftMotor.SetSmartCurrentLimit(10);
    m_liftMotor.EnableVoltageCompensation(12.0);

    m_shootMotor.SetInverted(false);
    m_shootMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_shootMotor.SetSmartCurrentLimit(10);
    m_shootMotor.EnableVoltageCompensation(12.0);
}

void AmpShooterSubsystem::Lift() {
    m_liftMotor.Set(0.2);
}

void AmpShooterSubsystem::Drop() {
    m_liftMotor.Set(-0.1);
}

void AmpShooterSubsystem::StopLift() {
    m_liftMotor.StopMotor();
}

bool AmpShooterSubsystem::IsLiftAtLimit() {
    return m_config.liftCurrentThreshold <= m_liftMotor.GetOutputCurrent();
}

void AmpShooterSubsystem::Shoot() {
    m_shootMotor.Set(0.0);
}

void AmpShooterSubsystem::StopShoot() {
    m_shootMotor.StopMotor();
}

void AmpShooterSubsystem::Extend() {
    m_servo.SetPulseTime(m_config.servo.extendPulseTime);
}

void AmpShooterSubsystem::Retract() {
    m_servo.SetPulseTime(m_config.servo.retractPulseTime);
}

bool AmpShooterSubsystem::IsNoteDetected() {
    return m_noteSensor.Get();
}
