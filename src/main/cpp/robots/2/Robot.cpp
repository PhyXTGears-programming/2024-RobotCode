// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Camera.h"
#include "RobotConfig.h"

#include "robots/2/auto/auto.h"
#include "robots/2/auto/load/SubsystemRegistry.h"
#include "robots/2/Constants.h"
#include "robots/2/Deploy.h"
#include "robots/2/Robot.h"

#include "robots/2/commands/ClimbUp.h"
#include "robots/2/commands/CloseGate.h"
#include "robots/2/commands/Commands.h"
#include "robots/2/commands/DriveTeleopCommand.h"
#include "robots/2/commands/IntakeSpeaker.h"
#include "robots/2/commands/OpenGate.h"

#include "external/cpptoml.h"

#include <iostream>
#include <fmt/core.h>

#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace ::robot2;

namespace auto_ = constants::autonomous;

void robot2::Robot::RobotInit() {
    frc::DigitalInput robotId(interface::k_robotId);

    // Expect LOW on DIO for robot 2.
    if (false != robotId.Get()) {
        fmt::println(stderr, "!!!!!");
        fmt::println(stderr, "Expected code for robot 2.  Found code for robot {}", ROBOT_ID);
        fmt::println(stderr, "!!!!!");
        abort();
    }

    std::shared_ptr<cpptoml::table> toml = nullptr;

    std::cout << std::endl << "Building config" << std::endl;

    try {
        toml = cpptoml::parse_file(deploy::GetRobotDirectory() + "/config.toml");
    } catch (cpptoml::parse_exception & ex) {
        // clang-format off
        std::cerr
            << "Unable to open config file: "
            << deploy::GetRobotDirectory()
            << "/config.toml"
            << std::endl
            << ex.what()
            << std::endl;

        abort();
        // clang-format on
    }

    std::cout << std::endl << "Building camera" << std::endl;

    m_cameraFront = camera::LoadAndStart(deploy::GetRobotDirectory() + "/camera-front.json", 320, 240, 20);
    m_cameraBack = camera::LoadAndStart(deploy::GetRobotDirectory() + "/camera-back.json", 320, 240, 20);

    if (m_cameraFront) {
        frc::CameraServer::GetServer().SetSource(*m_cameraFront);
        m_usingCameraFront = true;
    } else if (m_cameraBack) {
        frc::CameraServer::GetServer().SetSource(*m_cameraBack);
        m_usingCameraFront = false;
    }

    std::cout << std::endl << "Building joysticks" << std::endl;

    m_driverController = new frc::XboxController(0);
    m_operatorController = new frc::XboxController(1);

    std::cout << std::endl << "Building subsystems" << std::endl;

    m_bling = new BlingSubsystem();
    m_climb = new ClimbSubsystem(toml->get_table("climb"));
    m_drivetrain = new Drivetrain(toml->get_table("drivetrain"));
    m_gate = new GateSubsystem(toml->get_table("gate"));
    m_intake = new IntakeSubsystem(toml->get_table("intake"));
    m_speaker = new SpeakerShooterSubsystem(toml->get_table("speaker"));

    std::cout << std::endl << "Building commands" << std::endl;

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

    m_preheatSpeaker = cmd::PreheatSpeakerFar(m_speaker);

    m_shootSpeakerFar = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = true; }, {}),
        cmd::PreheatSpeakerFar(m_speaker),
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = false; }, {}),
        cmd::ShootSpeakerFar(m_intake, m_speaker).WithTimeout(2_s)
    );
    m_shootSpeakerNear = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = true; }, {}),
        cmd::PreheatSpeakerNear(m_speaker),
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = false; }, {}),
        cmd::ShootSpeakerNear(m_intake, m_speaker).WithTimeout(2_s)
    );

    m_climbUp = ClimbUp(m_climb, m_bling, m_operatorController).ToPtr();

    m_autoShootSpeakerAndStay = frc2::cmd::Sequence(
        cmd::PreheatSpeakerNear(m_speaker),
        cmd::ShootSpeakerNear(m_intake, m_speaker).WithTimeout(3_s)
    );

    m_autoShootSpeakerAndLeave = frc2::cmd::Sequence(
        cmd::PreheatSpeakerNear(m_speaker),
        cmd::ShootSpeakerNear(m_intake, m_speaker).WithTimeout(3_s),
        moveForwardsCommand(m_drivetrain)
    );

    m_autoShootTwo = frc2::cmd::Sequence(
        OpenGate(m_gate).ToPtr(),
        cmd::PreheatSpeakerNear(m_speaker),
        cmd::ShootSpeakerNear(m_intake, m_speaker).WithTimeout(2_s),
        frc2::cmd::Parallel(
            moveForwardsCommand(m_drivetrain),
            frc2::cmd::Sequence(
                frc2::cmd::Wait(1_s),
                IntakeSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(2.5_s)
            )
        ),
        frc2::cmd::Race(
            moveBackwardsCommand(m_drivetrain),
            frc2::cmd::Sequence(
                frc2::cmd::Wait(1_s),
                cmd::PreheatSpeakerFar(m_speaker).Repeatedly()
            )
        ).WithTimeout(4_s),
        cmd::ShootSpeakerNear(m_intake, m_speaker).WithTimeout(2_s)
    );

    SubsystemRegistry registry{ m_drivetrain, m_intake, m_speaker };

    m_autoBlueSubwoof3nR21 = loadPathFollowCommandFromFile(
        deploy::GetRobotDirectory() + "/subwoofer-speaker-3n-r21-blue.json",
        registry
    );
    m_autoRedSubwoof3nR21 = loadPathFollowCommandFromFile(
        deploy::GetRobotDirectory() + "/subwoofer-speaker-3n-r21-red.json",
        registry
    );

    m_chooser.SetDefaultOption(auto_::k_None, auto_::k_None);
    m_chooser.AddOption(auto_::k_ShootSpeakerAndStay, auto_::k_ShootSpeakerAndStay);
    m_chooser.AddOption(auto_::k_ShootSpeakerAndLeave, auto_::k_ShootSpeakerAndLeave);
    m_chooser.AddOption(auto_::k_ShootTwo, auto_::k_ShootTwo);
    //m_chooser.AddOption(auto_::k_FollowPath, auto_::k_FollowPath);
    m_chooser.AddOption(auto_::k_Blue3nR21, auto_::k_Blue3nR21);
    m_chooser.AddOption(auto_::k_Red3nR21,  auto_::k_Red3nR21);
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
void robot2::Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    frc::SmartDashboard::PutBoolean("Climb Locked?", m_climb->IsLockEngaged());

    if (m_driverController->GetBButtonPressed()) {
        m_drivetrain->ResetGyro();
    }

    if (m_driverController->GetRightBumperPressed()) {
        if (m_usingCameraFront && m_cameraBack) {
            m_usingCameraFront = false;
            frc::CameraServer::GetServer().SetSource(*m_cameraBack);
        } else if (!m_usingCameraFront && m_cameraFront) {
            m_usingCameraFront = true;
            frc::CameraServer::GetServer().SetSource(*m_cameraFront);
        }
    }

    if (m_speaker->IsNoteDetected()) {
        m_bling->NotifyNotePresent();
    } else {
        m_bling->NotifyNoteAbsent();
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
void robot2::Robot::AutonomousInit() {
    m_autoSelected = m_chooser.GetSelected();
    fmt::print("Auto selected: {}\n", m_autoSelected);

    if (auto_::k_None == m_autoSelected) {
        // Do nothing.
    } else if (auto_::k_ShootSpeakerAndStay == m_autoSelected) {
        m_autoShootSpeakerAndStay.Schedule();
    } else if (auto_::k_ShootSpeakerAndLeave == m_autoSelected) {
        m_autoShootSpeakerAndLeave.Schedule();
    } else if (auto_::k_ShootTwo == m_autoSelected) {
        m_autoShootTwo.Schedule();
    } else if (auto_::k_FollowPath == m_autoSelected) {
        m_autoPathTest.Schedule();
    } else if (auto_::k_Blue3nR21 == m_autoSelected) {
        m_autoBlueSubwoof3nR21.Schedule();
    } else if (auto_::k_Red3nR21 == m_autoSelected) {
        m_autoRedSubwoof3nR21.Schedule();
    }

    m_drivetrain->SetTurnBrake(true);
}

void robot2::Robot::AutonomousPeriodic() {
    m_drivetrain->UpdateOdometry();
}

void robot2::Robot::TeleopInit() {
    m_drivetrain->SetTurnBrake(true);

    m_driveTeleopCommand.Schedule();
    m_openGate.Schedule();
}

void robot2::Robot::TeleopPeriodic() {
    // Driver Controls
    if (m_driverController->GetYButtonPressed()) {
        m_drivetrain->ToggleFieldOriented();
    }

    // Operator Controls
    if (m_operatorController->GetAButtonPressed()) {
        m_intakeSpeaker.Schedule();
    } else if (m_operatorController->GetAButtonReleased()) {
        m_intakeSpeaker.Cancel();
    }

    if (m_operatorController->GetXButtonPressed()) {
        m_shootSpeakerFar.Schedule();
    } else if (m_operatorController->GetXButtonReleased()) {
        m_shootSpeakerFar.Cancel();
    }

    if (m_operatorController->GetYButtonPressed()) {
        m_shootSpeakerNear.Schedule();
    } else if (m_operatorController->GetYButtonReleased()) {
        m_shootSpeakerNear.Cancel();
    }

    if (m_operatorController->GetLeftBumperPressed()) {
        m_reverseSpeaker.Schedule();
    } else if (m_operatorController->GetLeftBumperReleased()) {
        m_reverseSpeaker.Cancel();
    }

    if (m_operatorController->GetRightBumperPressed()) {
        m_preheatSpeaker.Schedule();
    } else if (m_operatorController->GetRightBumperReleased()) {
        m_preheatSpeaker.Cancel();
    }

    if (0.1 < std::abs(m_operatorController->GetLeftY())) {
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

void robot2::Robot::DisabledInit() {}

void robot2::Robot::DisabledPeriodic() {}

void robot2::Robot::TestInit() {}

void robot2::Robot::TestPeriodic() {}

void robot2::Robot::SimulationInit() {}

void robot2::Robot::SimulationPeriodic() {}
