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

    cpptoml::option<double> servoExtendAngleDeg = table->get_qualified_as<double>("servo.extendAngleDeg");
    if (!servoExtendAngleDeg) {
        throw "Error: ampShooter cannot find toml property servo.extendAngleDeg";
    }
    m_config.servo.extendAngleDeg = *servoExtendAngleDeg;

    cpptoml::option<double> servoRetractAngleDeg = table->get_qualified_as<double>("servo.retractAngleDeg");
    if (!servoRetractAngleDeg) {
        throw "Error: ampShooter cannot find toml property servo.retractAngleDeg";
    }
    m_config.servo.retractAngleDeg = *servoRetractAngleDeg;

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
    m_servo.SetAngle(m_config.servo.extendAngleDeg);
}

void AmpShooterSubsystem::Retract() {
    m_servo.SetAngle(m_config.servo.retractAngleDeg);
}

bool AmpShooterSubsystem::IsNoteDetected() {
    return m_noteSensor.Get();
}
