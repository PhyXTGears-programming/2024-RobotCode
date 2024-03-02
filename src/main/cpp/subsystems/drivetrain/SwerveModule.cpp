// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/drivetrain/SwerveModule.h"
#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include "ctre/phoenix/sensors/AbsoluteSensorRange.h"
#include "rev/ControlType.h"
#include "rev/SparkRelativeEncoder.h"
#include "util/math.h"

#include <numbers>
#include <Constants.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>

using meters_per_turn = units::compound_unit<units::meters, units::inverse<units::turns>>;
using meters_per_turn_t = units::unit_t<meters_per_turn>;

#define ROBOT_ID 1

#if ROBOT_ID == 1
// FIXME: measure steer ratio and gear ratio of 2024 robot swerve modules.
static constexpr double MK1_DRIVE_RATIO = 1.0 / 1.0; // motor turns per gearbox turn
static constexpr double DRIVE_GEAR_RATIO = 1.0 / 1.0; //  gearbox turns per wheel turn
#elif ROBOT_ID == 2
// FIXME: measure steer ratio and gear ratio of 2024 robot swerve modules.
static constexpr double MK1_DRIVE_RATIO = 1.0 / 1.0; // motor turns per gearbox turn
static constexpr double DRIVE_GEAR_RATIO = 1.0 / 1.0; //  gearbox turns per wheel turn
#endif


SwerveModule::SwerveModule(
    int driveMotorCan,
    int turningMotorCan,
    int turningAbsEncoderCan,
    double absEncoderOffset,
    std::string_view name
) :
    m_name(name),
    m_absEncoderOffset(absEncoderOffset),
    m_driveMotor(driveMotorCan, rev::CANSparkLowLevel::MotorType::kBrushless),
    m_drivePid(m_driveMotor.GetPIDController()),
    m_driveEncoder(m_driveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    m_turningMotor(turningMotorCan, rev::CANSparkLowLevel::MotorType::kBrushless),
    m_turningPid(m_turningMotor.GetPIDController()),
    m_turningEncoder(m_turningMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    m_turningAbsEncoder(turningAbsEncoderCan)
{
    m_turningMotor.SetInverted(false);
    m_turningMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_turningMotor.SetSmartCurrentLimit(30);

    m_turningAbsEncoder.ConfigAbsoluteSensorRange(
        ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180
    );
    m_turningAbsEncoder.ConfigSensorDirection(false);       // FIXME: confirm direction during robot test.
    m_turningAbsEncoder.ConfigSensorInitializationStrategy(
        ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition
    );

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // this is the angele through an entire rotation (2* std::numbers::pu)
    // divided by the encoder resolution
    const double DEG_PER_PULSE = 0.087890625;
    m_turningAbsEncoder.ConfigFeedbackCoefficient(
        DEG_PER_PULSE * 2.0 * std::numbers::pi /* radians */ / 360.0 /* degress */,
        "radians",
        ctre::phoenix::sensors::SensorTimeBase::PerSecond,
        100.0
    );

    // FIXME: measure steer ratio and gear ratio of 2024 robot swerve modules.
    // 1.023 from experiment. measure (relHeading initialrelheading)  / (absheading initial abs heading)
    // after manually turning wheel in a full circle more turns = more accuracy
    static constexpr double MK1_STEER_RATIO = 1.023 * 11.30 / 1.0; // motor turns per gearbox turn
    static constexpr double STEER_GEAR_RATIO = 4.0 / 1.0; //  gearbox turns per wheel turn

    m_turningEncoder.SetPositionConversionFactor(
        2.0 * std::numbers::pi    //  radians per (any) turn
        / MK1_STEER_RATIO         //  gearbox turn per motor turn
        / STEER_GEAR_RATIO        //  wheel turn per gearbox turn
    );
    m_turningEncoder.SetVelocityConversionFactor(
        (2.0 *std::numbers::pi  / 1.0)  //  radians per any turn
        / MK1_STEER_RATIO              //  gearbox turn per motor turn
        / STEER_GEAR_RATIO             //  wheel turn per gearbox turn
        * (1.0 / 60.0)                  //  minute per second
    );

    m_turningPid.SetFeedbackDevice(m_turningEncoder);

    // FIXME: tune pids on robot.
    // Limit the pid controllers input range between -pi and pi and set the
    // input to be continuous.
    m_turningPid.SetP(0.4);
    m_turningPid.SetI(0.001);
    m_turningPid.SetIZone(std::numbers::pi / 8.0);  // 1/8 of 180 deg
    m_turningPid.SetD(0.0);
    //  feedforward sign does not reflect pid error will always push in one direction not to zero error
    m_turningPid.SetFF(0.0);
    m_turningPid.SetOutputRange(-1.0, 1.0);
    m_turningPid.SetPositionPIDWrappingEnabled(true);
    m_turningPid.SetPositionPIDWrappingMinInput(-std::numbers::pi);
    m_turningPid.SetPositionPIDWrappingMaxInput(std::numbers::pi);

    m_turningEncoder.SetPosition(m_turningAbsEncoder.GetPosition());

    // FIXME: confirm direction of drive motor on robot.
    m_driveMotor.SetInverted(false);
    m_driveMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_driveMotor.SetSmartCurrentLimit(40);

    m_driveEncoder.SetPositionConversionFactor(
        MK1_DRIVE_RATIO         //  gearbox turn per motor turn
        / DRIVE_GEAR_RATIO      //  wheel turn per gearbox turn
    );
    m_driveEncoder.SetVelocityConversionFactor(
        MK1_DRIVE_RATIO         //  gearbox turn per motor turn
        / DRIVE_GEAR_RATIO      //  wheel turn per gearbox turn
        * (1.0 / 60.0)          //  minute per second
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
        units::radian_t{ m_turningEncoder.GetPosition() }
    };
}


frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {units::meter_t{m_driveEncoder.GetPosition()},
            units::radian_t{m_turningEncoder.GetPosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& desiredState
) {
    frc::Rotation2d encoderRotation {
        units::radian_t{m_turningEncoder.GetPosition()}
    };
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
    ).to<double>();

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
        std::clamp((driveSpeed  / Constants::k_maxDriveSpeed).to<double>(), -1.0, 1.0)
    );
    */

    m_turningPid.SetReference(
        targetState.angle.Radians().to<double>(),
        rev::ControlType::kPosition
    );
}

void SwerveModule::ResetZeroTurn() {
    m_turningEncoder.SetPosition(m_turningAbsEncoder.GetPosition());
}

void SwerveModule::UpdateDashboard() {
    frc::SmartDashboard::PutNumber(
        std::string{m_name} + std::string{"/rel-heading"},
        RAD_2_DEG(m_turningEncoder.GetPosition())
    );
    frc::SmartDashboard::PutNumber(
        std::string{m_name} + std::string{"abs-heading"},
        RAD_2_DEG(m_turningAbsEncoder.GetPosition())
    );
    frc::SmartDashboard::PutNumber(
        std::string{m_name} + std::string{"/drive-speed"},
        m_driveEncoder.GetVelocity()
    );
}

