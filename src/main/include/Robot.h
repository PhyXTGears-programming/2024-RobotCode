// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "commands/DriveTeleopCommand.h"

#include "subsystems/amp_shooter/AmpShooter.h"
#include "subsystems/drivetrain/Drivetrain.h"
#include "subsystems/intake/Intake.h"
#include "subsystems/gate/Gate.h"

#include <string>

#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>

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
        const std::string kAutoNameDefault = "Default";
        const std::string kAutoNameCustom = "My Auto";
        std::string m_autoSelected;


        frc::XboxController * m_driverController = nullptr;

        AmpShooterSubsystem * m_amp = nullptr;
        Drivetrain * m_drivetrain = nullptr;
        GateSubsystem * m_gate = nullptr;
        IntakeSubsystem * m_intake = nullptr;

        DriveTeleopCommand * m_driveTeleopCommand = nullptr;
};
