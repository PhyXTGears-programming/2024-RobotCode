// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD liscense file in the root directory of this project.

#pragma once

#include "subsystems/drivetrain/Diagnostic.h"

#include <numbers>
#include <string_view>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/Phoenix.h>

#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/CANSparkMax.h>

using namespace std::literals::string_view_literals;

class SwerveModule {
public:
    SwerveModule(
        int driveMotorCan,
        int turningMotorCan,
        int turningAbsEncoderCan,
        double absEncoderOffset,
        std::string_view name = "swerve ??"sv
    );

    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();

    void SetDesiredState(const frc::SwerveModuleState& state);

    void ResetTurnPosition();

    units::radian_t GetTurnPosition();
    units::radian_t GetTurnAbsPosition();
    units::radian_t GetTurnAbsPositionRaw();

    void UpdateDashboard();
private:
    static constexpr double kWheelRadius = 0.508;
    static constexpr int kEncoderResolution = 4096;

    static constexpr auto kModuleMaxAnglarVelocity =
        std::numbers::pi * 1_rad_per_s; // radians per second 

    static constexpr auto kModuleMaxAnglarAcceleration =
        std::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

    std::string m_name;

    double m_absEncoderOffset = 0.0;

public:
    rev::CANSparkMax m_driveMotor;
    rev::SparkPIDController m_drivePid;
    rev::SparkRelativeEncoder m_driveEncoder;

    rev::CANSparkMax m_turningMotor;
    rev::SparkPIDController m_turningPid;
    rev::SparkRelativeEncoder m_turningEncoder;

    ctre::phoenix::sensors::CANCoder m_turningAbsEncoder;

    friend diagnostic::TestDrivetrain;
};

