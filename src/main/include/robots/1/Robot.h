// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "robots/1/subsystems/amp_shooter/AmpShooter.h"
#include "robots/1/subsystems/bling/Bling.h"
#include "robots/1/subsystems/climb/Climb.h"
#include "robots/1/subsystems/drivetrain/Drivetrain.h"
#include "robots/1/subsystems/intake/Intake.h"
#include "robots/1/subsystems/gate/Gate.h"
#include "robots/1/subsystems/speaker_shooter/SpeakerShooter.h"

#include <string>

#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace robot1 {
    using namespace ::robot1;

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

            AmpShooterSubsystem * m_amp = nullptr;
            BlingSubsystem * m_bling = nullptr;
            ClimbSubsystem * m_climb = nullptr;
            Drivetrain * m_drivetrain = nullptr;
            GateSubsystem * m_gate = nullptr;
            IntakeSubsystem * m_intake = nullptr;
            SpeakerShooterSubsystem * m_speaker = nullptr;

            frc2::CommandPtr m_driveTeleopCommand = frc2::cmd::None();

            frc2::CommandPtr m_closeGate = frc2::cmd::None();
            frc2::CommandPtr m_openGate = frc2::cmd::None();

            frc2::CommandPtr m_retractAmp = frc2::cmd::None();

            frc2::CommandPtr m_intakeSpeaker = frc2::cmd::None();
            frc2::CommandPtr m_reverseSpeaker = frc2::cmd::None();
            frc2::CommandPtr m_preheatSpeaker = frc2::cmd::None();
            frc2::CommandPtr m_shootSpeaker = frc2::cmd::None();
            frc2::CommandPtr m_shootSpeakerSlow = frc2::cmd::None();

            frc2::CommandPtr m_climbUp = frc2::cmd::None();

            frc2::CommandPtr m_autoShootSpeakerAndStay = frc2::cmd::None();

            // Auto 2: Shoot note in speaker and drive forward out of starting zone.
            frc2::CommandPtr m_autoShootSpeakerAndLeave = frc2::cmd::None();

            frc2::CommandPtr m_autoShootTwo = frc2::cmd::None();

            frc2::CommandPtr m_autoPathTest = frc2::cmd::None();
            frc2::CommandPtr m_autoBlueSubwoof2nRamp = frc2::cmd::None();
            frc2::CommandPtr m_autoBlueSubwoof3nRampCenter = frc2::cmd::None();

            bool m_isShootSpeakerInPreheat = false;
    };

}
