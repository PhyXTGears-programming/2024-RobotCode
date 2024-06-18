// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "robots/2/subsystems/bling/Bling.h"
#include "robots/2/subsystems/climb/Climb.h"
#include "robots/2/subsystems/drivetrain/Drivetrain.h"
#include "robots/2/subsystems/intake/Intake.h"
#include "robots/2/subsystems/gate/Gate.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"
#include "robots/2/subsystems/shooter_tilt/ShooterTilt.h"

#include <optional>
#include <string>

#include <cameraserver/CameraServer.h>

#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace robot2 {
    using namespace ::robot2;

    class Robot : public frc::TimedRobot {
        public:
            void RobotInit() override;
            void RobotPeriodic() override;
            void AutonomousInit() override;
            void AutonomousPeriodic() override;
            void TeleopInit() override;
            void TeleopPeriodic() override;
            void DisabledInit() override;
            void DisabledPeriodic() override;
            void TestInit() override;
            void TestPeriodic() override;
            void SimulationInit() override;
            void SimulationPeriodic() override;

        private:
            frc::SendableChooser<std::string> m_chooser;
            std::string m_autoSelected;


            frc::XboxController * m_driverController = nullptr;
            frc::XboxController * m_operatorController = nullptr;

            bool m_usingCameraFront = true;
            std::optional<cs::UsbCamera> m_cameraFront;
            std::optional<cs::UsbCamera> m_cameraBack;

            BlingSubsystem * m_bling = nullptr;
            ClimbSubsystem * m_climb = nullptr;
            Drivetrain * m_drivetrain = nullptr;
            GateSubsystem * m_gate = nullptr;
            IntakeSubsystem * m_intake = nullptr;
            SpeakerShooterSubsystem * m_speaker = nullptr;
            ShooterTiltSubsystem * m_tilt = nullptr;

            frc2::CommandPtr m_driveTeleopCommand = frc2::cmd::None();

            frc2::CommandPtr m_closeGate = frc2::cmd::None();
            frc2::CommandPtr m_openGate = frc2::cmd::None();

            frc2::CommandPtr m_intakeSpeaker  = frc2::cmd::None();
            frc2::CommandPtr m_overrideIntake = frc2::cmd::None();
            frc2::CommandPtr m_reverseSpeaker = frc2::cmd::None();
            frc2::CommandPtr m_preheatSpeaker = frc2::cmd::None();

            frc2::CommandPtr m_shootAmp         = frc2::cmd::None();
            frc2::CommandPtr m_shootSpeakerFar  = frc2::cmd::None();
            frc2::CommandPtr m_shootSpeakerNear = frc2::cmd::None();
            frc2::CommandPtr m_shootTrap        = frc2::cmd::None();

            frc2::CommandPtr m_tiltSpeaker = frc2::cmd::None();
            frc2::CommandPtr m_tiltStage   = frc2::cmd::None();

            frc2::CommandPtr m_climbUp = frc2::cmd::None();

            frc2::CommandPtr m_autoShootSpeakerAndStay = frc2::cmd::None();

            // Auto 2: Shoot note in speaker and drive forward out of starting zone.
            frc2::CommandPtr m_autoShootSpeakerAndLeave = frc2::cmd::None();

            frc2::CommandPtr m_autoShootTwo = frc2::cmd::None();

            frc2::CommandPtr m_autoPathTest = frc2::cmd::None();
            frc2::CommandPtr m_autoBlueSubwoof3nR21 = frc2::cmd::None();
            frc2::CommandPtr m_autoRedSubwoof3nR21  = frc2::cmd::None();
            frc2::CommandPtr m_autoRedSubBot3nR3C5  = frc2::cmd::None();
            frc2::CommandPtr m_autoRedSubBot1nC5    = frc2::cmd::None();
            frc2::CommandPtr m_autoRedSubBot0nR3    = frc2::cmd::None();
            frc2::CommandPtr m_autoBlueSubBot1nC5   = frc2::cmd::None();
            frc2::CommandPtr m_autoBlueSubBot0n     = frc2::cmd::None();
            frc2::CommandPtr m_autoBlueScatter      = frc2::cmd::None();

            bool m_isShootSpeakerInPreheat = false;
    };

}
