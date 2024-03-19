// Copyright (c) FIRST and other WPILib contributors.
// Open source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory fo this project.

#include "Constants.h"
#include "Interface.h"
#include "subsystems/drivetrain/Drivetrain.h"
#include "util/point.h"

#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/Timer.h>

using namespace std::literals::string_view_literals;

using units::degree_t;
using units::meters_per_second_t;
using units::radians_per_second_t;
using units::radian_t;
using units::second_t;

Drivetrain::Drivetrain(std::shared_ptr<cpptoml::table> table) {
    double turnP = 0.0;
    double turnI = 0.0;
    double turnD = 0.0;

    {
        cpptoml::option<double> kP = table->get_qualified_as<double>("swerve.turn.kP");

        if (!kP) {
            std::cerr << "Error: drivetrain cannot find toml property swerve.turn.kP" << std::endl;
            throw "error";
        }

        turnP = *kP;
    }

    {
        cpptoml::option<double> kI = table->get_qualified_as<double>("swerve.turn.kI");

        if (!kI) {
            std::cerr << "Error: drivetrain cannot find toml property swerve.turn.kI" << std::endl;
            throw "error";
        }

        turnI = *kI;
    }

    {
        cpptoml::option<double> kD = table->get_qualified_as<double>("swerve.turn.kD");

        if (!kD) {
            std::cerr << "Error: drivetrain canot find toml property swerve.turn.kD" << std::endl;
            throw "error";
        }

        turnD = *kD;
    }

    SwerveModule::PidConfig turnPidConfig(turnP, turnI, turnD);

    {
        cpptoml::option<double> frontLeftAbsEncoderOffset =
            table->get_qualified_as<double>("frontLeftAbsEncoderOffset");

        if (!frontLeftAbsEncoderOffset) {
            std::cerr << "Error: drivetrain cannot find toml property frontLeftAbsEncoderOffset" << std::endl;
            throw "error";
        }

        m_frontLeft = new SwerveModule(
            interface::drive::k_frontLeftDrive,
            interface::drive::k_frontLeftTurn,
            interface::drive::k_frontLeftEncoder,
            units::degree_t(*frontLeftAbsEncoderOffset),
            turnPidConfig,
            "front-left"
        );
    }

    {
        cpptoml::option<double> frontRightAbsEncoderOffset =
            table->get_qualified_as<double>("frontRightAbsEncoderOffset");

        if (!frontRightAbsEncoderOffset) {
            std::cerr << "Error: drivetrain cannot find toml property frontRightAbsEncoderOffset" << std::endl;
            throw "error";
        }

        m_frontRight = new SwerveModule(
            interface::drive::k_frontRightDrive,
            interface::drive::k_frontRightTurn,
            interface::drive::k_frontRightEncoder,
            units::degree_t(*frontRightAbsEncoderOffset),
            turnPidConfig,
            "front-right"
        );
    }

    {
        cpptoml::option<double> backLeftAbsEncoderOffset =
            table->get_qualified_as<double>("backLeftAbsEncoderOffset");

        if (!backLeftAbsEncoderOffset) {
            std::cerr << "Error: drivetrain cannot find toml property backLeftAbsEncoderOffset" << std::endl;
            throw "error";
        }

        m_backLeft = new SwerveModule(
            interface::drive::k_backLeftDrive,
            interface::drive::k_backLeftTurn,
            interface::drive::k_backLeftEncoder,
            units::degree_t(*backLeftAbsEncoderOffset),
            turnPidConfig,
            "back-left"
        );
    }

    {
        cpptoml::option<double> backRightAbsEncoderOffset =
            table->get_qualified_as<double>("backRightAbsEncoderOffset");

        if (!backRightAbsEncoderOffset) {
            std::cerr << "Error: drivetrain cannot find toml property backRightAbsEncoderOffset" << std::endl;
            throw "error";
        }

        m_backRight = new SwerveModule(
            interface::drive::k_backRightDrive,
            interface::drive::k_backRightTurn,
            interface::drive::k_backRightEncoder,
            units::degree_t(*backRightAbsEncoderOffset),
            turnPidConfig,
            "back-right"
        );
    }

    m_gyro.Reset();

    frc::Timer deadline;
    deadline.Start();

    while (m_gyro.IsCalibrating()) {
        if (deadline.AdvanceIfElapsed(30_s)) {
            std::cerr << "Error: Drivetrain: navx calibration deadline elapsed" << std::endl;
            break;
        }
    }

    ResetGyro();

    m_odometry = new frc::SwerveDriveOdometry(
        m_kinematics,
        frc::Rotation2d(GetHeading()),
        {
            m_frontLeft->GetPosition(),
            m_frontRight->GetPosition(),
            m_backLeft->GetPosition(),
            m_backRight->GetPosition()
        }
    );
}

void Drivetrain::Drive(
    meters_per_second_t forwardSpeed,
    meters_per_second_t strafeSpeed,
    radians_per_second_t turnSpeed,
    bool fieldRelative,
    second_t period = 20_ms
) {
    auto chassisSpeeds =
        fieldRelative
        ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                forwardSpeed,
                strafeSpeed,
                turnSpeed,
                frc::Rotation2d(GetHeading())
            )
        : frc::ChassisSpeeds(forwardSpeed, strafeSpeed, turnSpeed);

    auto states = m_kinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds::Discretize(chassisSpeeds, period)
    );

    m_kinematics.DesaturateWheelSpeeds(&states, constants::k_maxDriveSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft->SetDesiredState(fl);
    m_frontRight->SetDesiredState(fr);
    m_backLeft->SetDesiredState(bl);
    m_backRight->SetDesiredState(br);
}

void Drivetrain::Periodic() {
    m_frontLeft->Periodic();
    m_frontRight->Periodic();
    m_backLeft->Periodic();
    m_backRight->Periodic();

    m_frontLeft->UpdateDashboard();
    m_frontRight->UpdateDashboard();
    m_backLeft->UpdateDashboard();
    m_backRight->UpdateDashboard();

    frc::SmartDashboard::PutBoolean(
        "Is Field Oriented?",
        IsFieldOriented()
    );

    frc::SmartDashboard::PutNumber(
        "Front Left Distance (m)",
        m_frontLeft->GetPosition().distance.value()
    );
}

bool Drivetrain::IsFieldOriented() {
    return m_isFieldOriented;
}

void Drivetrain::SetFieldOriented(bool isFieldOriented) {
    m_isFieldOriented = isFieldOriented;
}

void Drivetrain::ToggleFieldOriented() {
    m_isFieldOriented = !m_isFieldOriented;
}
 
void Drivetrain::ResetGyro() {
    m_gyroOffset = -m_gyro.GetYaw();
}

radian_t Drivetrain::GetHeading() {
    return -degree_t(m_gyro.GetYaw() + m_gyroOffset);
}

void Drivetrain::UpdateOdometry() {
    m_odometry->Update(
        frc::Rotation2d(GetHeading()),
        {
            m_frontLeft->GetPosition(),
            m_frontRight->GetPosition(),
            m_backLeft->GetPosition(),
            m_backRight->GetPosition()
        }
    );
}

void Drivetrain::ResetPosition(){
    m_odometry->ResetPosition(
        frc::Rotation2d(GetHeading()),
        {
            m_frontLeft->GetPosition(),
            m_frontRight->GetPosition(),
            m_backLeft->GetPosition(),
            m_backRight->GetPosition(),
        },
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_rad))
    );
}

void Drivetrain::SetPosition(radian_t heading, frc::Pose2d toPose) {
    m_odometry->ResetPosition(
        frc::Rotation2d(heading),
        {
            m_frontLeft->GetPosition(),
            m_frontRight->GetPosition(),
            m_backLeft->GetPosition(),
            m_backRight->GetPosition(),
        },
        toPose
    );
}

void Drivetrain::SetPose(frc::Pose2d toPose) {
    m_odometry->ResetPosition(
        frc::Rotation2d(GetHeading()),
        {
            m_frontLeft->GetPosition(),
            m_frontRight->GetPosition(),
            m_backLeft->GetPosition(),
            m_backRight->GetPosition(),
        },
        toPose
    );
}

Point Drivetrain::GetChassisPosition() {
    frc::Translation2d translation = m_odometry->GetPose().Translation();

    return Point(translation.X().value(), translation.Y().value());
}


