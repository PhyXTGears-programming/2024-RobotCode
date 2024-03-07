// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Interface.h"
#include "external/cpptoml.h"
#include "subsystems/drivetrain/Diagnostic.h"
#include "subsystems/drivetrain/SwerveModule.h"
#include "util/point.h"

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include <AHRS.h>


/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain(std::shared_ptr<cpptoml::table> table);

        void Drive(
            units::meters_per_second_t xSpeed,
            units::meters_per_second_t ySpeed,
            units::radians_per_second_t rot,
            bool fieldRelative,
            units::second_t period
        );

        void Periodic() override;

        void UpdateOdometry();

        void ResetGyro();

        double GetHeading();

        void ResetPosition();

        void SetPosition(frc::Pose2d toPose);

        Point GetChassisPosition();

    private:
        // FIXME: measure dimensions between wheel axles and update.
        frc::Translation2d m_frontLeftLocation { -0.222_m, +0.248_m };
        frc::Translation2d m_frontRightLocation{ +0.222_m, +0.248_m };
        frc::Translation2d m_backLeftLocation  { -0.222_m, -0.248_m };
        frc::Translation2d m_backRightLocation { +0.222_m, -0.248_m };

    public:
        SwerveModule * m_frontLeft = nullptr;

        SwerveModule * m_frontRight = nullptr; 

        SwerveModule * m_backLeft = nullptr; 

        SwerveModule * m_backRight = nullptr; 

        AHRS m_gyro{ frc::SPI::kMXP };
        double m_gyroOffset = 0.0;

        frc::SwerveDriveKinematics<4> m_kinematics{
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation
        };

        frc::SwerveDriveOdometry<4> m_odometry{
            m_kinematics,
            m_gyro.GetRotation2d(),
            {
                m_frontLeft->GetPosition(),
                m_frontRight->GetPosition(),
                m_backLeft->GetPosition(),
                m_backRight->GetPosition()
            }
        };

        friend diagnostic::TestDrivetrain;
};
