// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "external/cpptoml.h"
#include "robots/1/Constants.h"
#include "robots/1/subsystems/drivetrain/DiagnosticDecl.h"
#include "robots/1/subsystems/drivetrain/SwerveModule.h"
#include "util/point.h"

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include <AHRS.h>

using robot1::constants::drive::k_numberOfSwerveModules;


namespace robot1 {

    /**
    * Represents a swerve drive style drivetrain.
    */
    class Drivetrain : public frc2::SubsystemBase {
        public:
            Drivetrain(std::shared_ptr<cpptoml::table> table);

            void Drive(
                units::meters_per_second_t forwardSpeed,
                units::meters_per_second_t strafeSpeed,
                units::radians_per_second_t turnSpeed,
                bool fieldRelative,
                units::second_t period
            );

            void Periodic() override;

            void UpdateOdometry();

            bool IsFieldOriented();
            void SetFieldOriented(bool isFieldOriented);
            void ToggleFieldOriented();

            void ResetGyro();
            void ResetGyroToHeading(units::radian_t heading);

            units::radian_t GetHeading();

            void ResetPosition();

            void SetPosition(units::radian_t heading, frc::Pose2d toPose);

            void SetPose(frc::Pose2d toPose);

            Point GetChassisPosition();

        private:
            frc::Translation2d m_frontLeftLocation { +0.287_m, +0.287_m };
            frc::Translation2d m_frontRightLocation{ +0.287_m, -0.287_m };
            frc::Translation2d m_backLeftLocation  { -0.287_m, +0.287_m };
            frc::Translation2d m_backRightLocation { -0.287_m, -0.287_m };

            SwerveModule * m_frontLeft = nullptr;
            SwerveModule * m_frontRight = nullptr;
            SwerveModule * m_backLeft = nullptr;
            SwerveModule * m_backRight = nullptr;

            AHRS m_gyro{ frc::SPI::kMXP };
            double m_gyroOffset = 0.0;

            bool m_isFieldOriented = true;

            frc::SwerveDriveKinematics<k_numberOfSwerveModules> m_kinematics{
                m_frontLeftLocation,
                m_frontRightLocation,
                m_backLeftLocation,
                m_backRightLocation
            };

            frc::SwerveDriveOdometry<k_numberOfSwerveModules> * m_odometry;

            friend class robot1::diagnostic::TestDrivetrain;
    };

}
