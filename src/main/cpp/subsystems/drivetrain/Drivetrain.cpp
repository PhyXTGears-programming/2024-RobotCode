//Copyright (c) FIRST and other WPILib contributors.
//Open source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory fo this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>

#include "Constants.h"
#include "Interface.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "subsystems/drivetrain/Drivetrain.h"
#include "util/point.h"

using namespace std::literals::string_view_literals;

using units::meters_per_second_t;
using units::radians_per_second_t;
using units::second_t;

Drivetrain::Drivetrain(std::shared_ptr<cpptoml::table> table) {
    cpptoml::option<double> frontLeftAbsEncoderOffset =
        table->get_qualified_as<double>("frontLeftAbsEncoderOffset");

    if (!frontLeftAbsEncoderOffset) {
        throw "Error: drivetrain cannot find toml property frontLeftAbsEncoderOffset";
    }

    m_frontLeft = new SwerveModule(
        Interface::k_drivetrainFrontLeftDrive,
        Interface::k_drivetrainFrontLeftSteer,
        Interface::k_drivetrainFrontLeftEncoder,
        *frontLeftAbsEncoderOffset,
        "front-left"
    );

    cpptoml::option<double> frontRightAbsEncoderOffset =
        table->get_qualified_as<double>("frontRightAbsEncoderOffset");

    if (!frontRightAbsEncoderOffset) {
        throw "Error: drivetrain cannot find toml property frontRightAbsEncoderOffset";
    }

    m_frontRight = new SwerveModule(
        Interface::k_drivetrainFrontRightDrive,
        Interface::k_drivetrainFrontRightSteer,
        Interface::k_drivetrainFrontRightEncoder,
        *frontRightAbsEncoderOffset,
        "front-right"
    );

    cpptoml::option<double> backLeftAbsEncoderOffset =
        table->get_qualified_as<double>("backLeftAbsEncoderOffset");

    if (!backLeftAbsEncoderOffset) {
        throw "Error: drivetrain cannot find toml property backLeftAbsEncoderOffset";
    }

    m_backLeft = new SwerveModule(
        Interface::k_drivetrainBackLeftDrive,
        Interface::k_drivetrainBackLeftSteer,
        Interface::k_drivetrainBackLeftEncoder,
        *backLeftAbsEncoderOffset,
        "back-left"
    );

    cpptoml::option<double> backRightAbsEncoderOffset =
        table->get_qualified_as<double>("backRightAbsEncoder");

    if (!backRightAbsEncoderOffset) {
        throw "Error: drivetrain cannot find toml property backRightAbsEncoderOffset";
    }

    m_backRight = new SwerveModule(
        Interface::k_drivetrainBackRightDrive,
        Interface::k_drivetrainBackRightSteer,
        Interface::k_drivetrainBackRightEncoder,
        *backRightAbsEncoderOffset,
        "back-right"
    );

    m_gyro.Reset();

    while (m_gyro.IsCalibrating());

    ResetGyro();

}

void Drivetrain::Drive(
    meters_per_second_t xSpeed,
    meters_per_second_t ySpeed,
    radians_per_second_t rot,
    bool fieldRelative,
    second_t period = 20_ms
) {
    auto chassisSpeeds =
        fieldRelative
        ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rot,
                m_gyro.GetRotation2d()
            )
        : frc::ChassisSpeeds(xSpeed, ySpeed, rot);

    auto states = m_kinematics.ToSwerveModuleStates(
            frc::ChassisSpeeds::Discretize(chassisSpeeds, period)
    );

    m_kinematics.DesaturateWheelSpeeds(&states, Constants::k_maxDriveSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft->SetDesiredState(fl);
    m_frontRight->SetDesiredState(fr);
    m_backLeft->SetDesiredState(bl);
    m_backRight->SetDesiredState(br);
}

void Drivetrain::Periodic() {
   m_frontLeft->UpdateDashboard();
   m_frontRight->UpdateDashboard();
   m_backLeft->UpdateDashboard();
   m_backRight->UpdateDashboard();
}
 
void Drivetrain::ResetGyro() {
    m_gyroOffset = -m_gyro.GetYaw();
}

double Drivetrain::GetHeading() {
    // TODO: import code from noodlebot
    return m_gyro.GetYaw() + m_gyroOffset;
}

void Drivetrain::UpdateOdometry() {
    // TODO: import code from noodlebot
    m_odometry.Update(
        m_gyro.GetRotation2d(),
        { m_frontLeft->GetPosition(),
            m_frontRight->GetPosition(),
            m_backLeft->GetPosition(),
            m_backRight->GetPosition()
        }
    );
}
void Drivetrain::resetPosition(){
    m_odometry.ResetPosition(
        m_gyro.GetRotation2d(),
        {
          m_frontLeft->GetPosition(),
          m_frontRight->GetPosition(),
          m_backLeft->GetPosition(),
          m_backRight->GetPosition(),
        },
        frc::Pose2d(0_m,0_m, frc::Rotation2d(0_rad))
    );
}

 void Drivetrain::setPosition(frc::Pose2d toPose) {
     m_odometry.ResetPosition(
        m_gyro.GetRotation2d(),
        {
          m_frontLeft->GetPosition(),
          m_frontRight->GetPosition(),
          m_backLeft->GetPosition(),
          m_backRight->GetPosition(),
        },
        toPose
    );
 }

Point Drivetrain::getChassisPosition() {
    frc::Translation2d translation = m_odometry.GetPose().Translation();

    return Point(translation.X().to<double>(), translation.Y().to<double>());
}


