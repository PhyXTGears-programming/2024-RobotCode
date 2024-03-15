// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "commands/ClimbUp.h"
#include "commands/CloseGate.h"
#include "commands/DriveTeleopCommand.h"
#include "commands/IntakeSpeaker.h"
#include "commands/OpenGate.h"
#include "commands/PreheatSpeaker.h"
#include "commands/ShootSpeaker.h"

#include "external/cpptoml.h"

#include <iostream>
#include <fmt/core.h>

#include <cameraserver/CameraServer.h>

#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

void Robot::RobotInit() {
    std::shared_ptr<cpptoml::table> toml = nullptr;

    try {
        toml = cpptoml::parse_file(frc::filesystem::GetDeployDirectory() + "/config.toml");
    } catch (cpptoml::parse_exception & ex) {
        // clang-format off
        std::cerr
            << "Unable to open config file: deploy/config.toml" << std::endl
            << ex.what() << std::endl;

        abort();
        // clang-format on
    }

    frc::CameraServer::StartAutomaticCapture();

    m_driverController = new frc::XboxController(0);
    m_operatorController = new frc::XboxController(1);

    m_amp = new AmpShooterSubsystem(toml->get_table("amp"));
    m_bling = new BlingSubsystem();
    m_climb = new ClimbSubsystem(toml->get_table("climb"));
    m_drivetrain = new Drivetrain(toml->get_table("drivetrain"));
    m_gate = new GateSubsystem(toml->get_table("gate"));
    m_intake = new IntakeSubsystem(toml->get_table("intake"));
    m_speaker = new SpeakerShooterSubsystem(toml->get_table("speaker"));

    m_driveTeleopCommand = DriveTeleopCommand(m_drivetrain, m_driverController).ToPtr();

    m_closeGate = CloseGate(m_gate).ToPtr().WithName("Close Gate");
    frc::SmartDashboard::PutData("Close Gate", m_closeGate.get());
    m_openGate = OpenGate(m_gate).ToPtr();

    m_intakeSpeaker = IntakeSpeaker(m_intake, m_speaker).ToPtr();
    m_reverseSpeaker = frc2::cmd::StartEnd(
        [this] () { m_intake->ReverseSpeakerShooter(); },
        [this] () { m_intake->Stop(); },
        { m_intake }
    );
    m_shootSpeaker = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = true; }, {}),
        PreheatSpeaker(m_speaker).ToPtr(),
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = false; }, {}),
        ShootSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(2_s)
    );

    m_climbUp = ClimbUp(m_climb, m_operatorController).ToPtr();

    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    frc::SmartDashboard::PutBoolean("Climb Locked?", m_climb->IsLockEngaged());

    if (m_driverController->GetBButtonPressed()) {
        m_drivetrain->ResetGyro();
    }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString("Auto Selector",
    //     kAutoNameDefault);
    fmt::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoNameCustom) {
        // Custom Auto goes here
    } else {
        // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic() {
    if (m_autoSelected == kAutoNameCustom) {
        // Custom Auto goes here
    } else {
        // Default Auto goes here
    }
}

void Robot::TeleopInit() {
    m_driveTeleopCommand.Schedule();
    m_openGate.Schedule();
}

void Robot::TeleopPeriodic() {
    // Driver Controls


    // Operator Controls
    if (m_operatorController->GetAButtonPressed()) {
        m_intakeSpeaker.Schedule();
    } else if (m_operatorController->GetAButtonReleased()) {
        m_intakeSpeaker.Cancel();
    }

    if (m_operatorController->GetXButtonPressed()) {
        m_shootSpeaker.Schedule();
    } else if (m_operatorController->GetXButtonReleased() && m_isShootSpeakerInPreheat) {
        m_shootSpeaker.Cancel();
    }

    if (m_operatorController->GetLeftBumperPressed()) {
        m_reverseSpeaker.Schedule();
    } else if (m_operatorController->GetLeftBumperReleased()) {
        m_reverseSpeaker.Cancel();
    }

    if (0.1 > std::abs(m_operatorController->GetLeftY())) {
        m_climbUp.Schedule();
    } else {
        m_climbUp.Cancel();
    }

    switch (m_operatorController->GetPOV(0)) {
        case 0:
            break;
        case 90:
            m_climb->Lock();
            break;
        case 180:
            break;
        case 270:
            m_climb->Unlock();
            break;
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
