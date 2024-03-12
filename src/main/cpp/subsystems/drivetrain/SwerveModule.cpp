// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "subsystems/drivetrain/SwerveModule.h"
#include "util/math.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>

#include <rev/ControlType.h>

#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

using meters_per_turn = units::compound_unit<units::meters, units::inverse<units::turns>>;
using meters_per_turn_t = units::unit_t<meters_per_turn>;

SwerveModule::SwerveModule(
    int driveMotorCan,
    int turningMotorCan,
    int turningAbsEncoderCan,
    units::radian_t absEncoderOffset,
    PidConfig const & turnConfig,
    std::string_view name
) :
    m_name(name),
    m_absEncoderOffset(absEncoderOffset),
    m_driveMotor(driveMotorCan, rev::CANSparkMax::MotorType::kBrushless),
    m_drivePid(m_driveMotor.GetPIDController()),
    m_driveEncoder(m_driveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    m_turningMotor(turningMotorCan, rev::CANSparkMax::MotorType::kBrushless),
    m_turningPid(m_turningMotor.GetPIDController()),
    m_turningEncoder(m_turningMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    m_turningAbsEncoder(turningAbsEncoderCan),
    m_turningAbsPositionSignal(m_turningAbsEncoder.GetAbsolutePosition())
{
    m_turningMotor.SetInverted(true);
    m_turningMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_turningMotor.SetSmartCurrentLimit(30);

    ctre::phoenix6::configs::CANcoderConfiguration configCanCoder{};
    ctre::phoenix6::configs::MagnetSensorConfigs configMagnetSensor{};
    configMagnetSensor
        .WithAbsoluteSensorRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf)
        .WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

    configCanCoder.WithMagnetSensor(configMagnetSensor);

    m_turningAbsEncoder.GetConfigurator().Apply(configCanCoder);

    m_turningEncoder.SetPositionConversionFactor(
        2.0 * std::numbers::pi                          // radians per motor turn
        / constants::drive::k_turnWheelPerMotorRatio    // motor turn per wheel turn
    );
    m_turningEncoder.SetVelocityConversionFactor(
        (2.0 * std::numbers::pi / 1.0)                  //  radians per motor turn
        / constants::drive::k_turnWheelPerMotorRatio    // motor turn per wheel turn
        * (1.0 / 60.0)                                  //  minute per second
    );

    m_turningPid.SetFeedbackDevice(m_turningEncoder);

    // Limit the pid controllers input range between -pi and pi and set the
    // input to be continuous.
    m_turningPid.SetP(turnConfig.kP);
    m_turningPid.SetI(turnConfig.kI);
    m_turningPid.SetIZone(std::numbers::pi / 8.0);  // 1/8 of 180 deg
    m_turningPid.SetD(turnConfig.kD);
    //  feedforward sign does not reflect pid error will always push in one direction not to zero error
    m_turningPid.SetFF(0.0);
    m_turningPid.SetOutputRange(-1.0, 1.0);
    m_turningPid.SetPositionPIDWrappingEnabled(true);
    m_turningPid.SetPositionPIDWrappingMinInput(-std::numbers::pi);
    m_turningPid.SetPositionPIDWrappingMaxInput(std::numbers::pi);

    ResetTurnPosition();

    // FIXME: confirm direction of drive motor on robot.
    m_driveMotor.SetInverted(false);
    m_driveMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_driveMotor.SetSmartCurrentLimit(40);

    m_driveEncoder.SetPositionConversionFactor(
        constants::drive::k_driveWheelPerMotorRatio
        * (std::numbers::pi * constants::drive::k_wheelDiameter.value() / 1.0)  // meter per turn
    );
    m_driveEncoder.SetVelocityConversionFactor(
        constants::drive::k_driveWheelPerMotorRatio
        * (std::numbers::pi * constants::drive::k_wheelDiameter.value() / 1.0)  // meter per turn
        * (1.0 / 60.0)                                                          //  minute per second
    );

    m_drivePid.SetFeedbackDevice(m_driveEncoder);

    // Drive pid parameters
    m_drivePid.SetP(1.0);
    m_drivePid.SetI(0.0);
    m_drivePid.SetIZone(0.0);
    m_drivePid.SetD(0.0);
    m_drivePid.SetFF(0.0);

    m_driveEncoder.SetPosition(0.0);
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {
        units::meters_per_second_t{ m_driveEncoder.GetVelocity() },
        GetTurnPosition()
    };
}


frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {
        units::meter_t{m_driveEncoder.GetPosition()},
        GetTurnPosition()
    };
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& desiredState
) {
    frc::Rotation2d encoderRotation{ GetTurnPosition() };

    // optimize the reference state to avoid spinning further than 90 degrees
    auto targetState = frc::SwerveModuleState::Optimize(
        desiredState,
        encoderRotation
    );

    //  scale speed by cosine of angel error this scales down movement
    //  perpendicular to the desired direction of travel that can occur when
    //  modules change directions. this results in smoother driving
    targetState.speed *= (targetState.angle - encoderRotation).Cos();

    // FIXME: measure wheel diameter.
    constexpr units::meter_t WHEEL_DIAMETER = 3.899_in;
    constexpr meters_per_turn_t WHEEL_DISTANCE_CONVERSION_FACTOR =
        WHEEL_DIAMETER
        * std::numbers::pi
        / 1.0_tr;
    constexpr double DRIVE_WHEEL_TO_MOTOR_RATIO = 1.0_tr / 6.12_tr;

    // units are actually native units per 100ms no units exist for
    // pulses/ticks/ native untis so let the units lib verify  the dims=ensionmath
    // up to turns per second
    double driveSpeed = (
        targetState.speed
        / WHEEL_DISTANCE_CONVERSION_FACTOR /* wheel turn per meter */
        / DRIVE_WHEEL_TO_MOTOR_RATIO       /* motor turn per wheel turn */
    ).value();

    // Set drive speed velocity.
    m_drivePid.SetReference(
        driveSpeed,
        rev::ControlType::kVelocity,
        0,
        std::copysign(10.0, driveSpeed)
    );

    /* TODO: compare performance of percent speed to pid velocity control.
    // Set drive speed percent.
    m_driveMotor.Set(
        std::clamp((driveSpeed  / constants::k_maxDriveSpeed).value(), -1.0, 1.0)
    );
    */

    m_turningPid.SetReference(
        targetState.angle.Radians().value(),
        rev::ControlType::kPosition
    );
}

void SwerveModule::ResetTurnPosition() {
    m_turningEncoder.SetPosition(
        GetTurnAbsPosition().convert<units::radian>().value()
    );
}

units::radian_t SwerveModule::GetTurnPosition() {
    return units::radian_t{ m_turningEncoder.GetPosition() };
}

units::radian_t SwerveModule::GetTurnAbsPosition() {
    auto position = units::math::abs(
        units::math::fmod(
            m_turningAbsPositionSignal.GetValue()
                + m_absEncoderOffset
                + 180_deg,
            360_deg
        )
    ) - 180_deg;

    return position.convert<units::radian>();
}

units::radian_t SwerveModule::GetTurnAbsPositionRaw() {
    return m_turningAbsPositionSignal.GetValue().convert<units::radian>();
}

void SwerveModule::UpdateDashboard() {
    frc::SmartDashboard::PutNumber(
        std::string{m_name} + std::string{"/rel-heading"},
        GetTurnPosition().convert<units::degree>().value()
    );
    frc::SmartDashboard::PutNumber(
        std::string{m_name} + std::string{"abs-heading"},
        GetTurnAbsPosition().convert<units::degree>().value()
    );
    frc::SmartDashboard::PutNumber(
        std::string{m_name} + std::string{"/drive-speed"},
        m_driveEncoder.GetVelocity()
    );
}

