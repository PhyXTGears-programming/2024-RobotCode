// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "auto/auto.h"
#include "auto/load/SubsystemRegistry.h"
#include "Constants.h"
#include "Robot.h"

#include "commands/ClimbUp.h"
#include "commands/CloseGate.h"
#include "commands/DriveTeleopCommand.h"
#include "commands/IntakeSpeaker.h"
#include "commands/OpenGate.h"
#include "commands/PreheatSpeaker.h"
#include "commands/PreheatSpeakerSlow.h"
#include "commands/RetractAmp.h"
#include "commands/ShootSpeaker.h"
#include "commands/ShootSpeakerSlow.h"

#include "external/cpptoml.h"

#include <iostream>
#include <fmt/core.h>

#include <cameraserver/CameraServer.h>

#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <wpi/json.h>
#include <wpi/MemoryBuffer.h>

namespace auto_ = constants::autonomous;

void Robot::RobotInit() {
    std::shared_ptr<cpptoml::table> toml = nullptr;

    std::cout << std::endl << "Building config" << std::endl;

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

    std::cout << std::endl << "Building camera" << std::endl;

    try {
        std::error_code ec;
        std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
            wpi::MemoryBuffer::GetFile(
                frc::filesystem::GetDeployDirectory() + "/camera.json",
                ec
            );

        if (nullptr == fileBuffer || ec) {
            std::cerr << "Error: Robot: unable to load camera json" << std::endl;
           
            auto camera = frc::CameraServer::StartAutomaticCapture();
            camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
            camera.SetResolution(320, 240);
            camera.SetFPS(20);
            frc::CameraServer::GetServer().SetSource(camera);
        } else {
            wpi::json cameraJson = wpi::json::parse(fileBuffer->begin(), fileBuffer->end());

            auto camera = frc::CameraServer::StartAutomaticCapture();
            camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
            camera.SetConfigJson(cameraJson);
            frc::CameraServer::GetServer().SetSource(camera);
        }
    } catch (...) {
        std::cerr << "Error: Robot: unknown exception while configuring camera" << std::endl;
    }

    std::cout << std::endl << "Building joysticks" << std::endl;

    m_driverController = new frc::XboxController(0);
    m_operatorController = new frc::XboxController(1);

    std::cout << std::endl << "Building subsystems" << std::endl;

    m_amp = new AmpShooterSubsystem(toml->get_table("amp"));
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

    m_retractAmp = RetractAmp(m_amp).ToPtr();

    m_intakeSpeaker = IntakeSpeaker(m_intake, m_speaker).ToPtr();
    m_reverseSpeaker = frc2::cmd::StartEnd(
        [this] () { m_intake->ReverseSpeakerShooter(); },
        [this] () { m_intake->Stop(); },
        { m_intake }
    );
    m_preheatSpeaker = PreheatSpeaker(m_speaker).ToPtr();
    m_shootSpeaker = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = true; }, {}),
        PreheatSpeaker(m_speaker).ToPtr(),
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = false; }, {}),
        ShootSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(2_s)
    );
    m_shootSpeakerSlow = frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = true; }, {}),
        PreheatSpeakerSlow(m_speaker).ToPtr(),
        frc2::cmd::RunOnce([this] () { m_isShootSpeakerInPreheat = false; }, {}),
        ShootSpeakerSlow(m_intake, m_speaker).ToPtr().WithTimeout(2_s)
    );

    m_climbUp = ClimbUp(m_climb, m_bling, m_operatorController).ToPtr();

    m_autoShootSpeakerAndStay = frc2::cmd::Sequence(
        PreheatSpeaker(m_speaker).ToPtr(),
        ShootSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(3_s)
    );

    m_autoShootSpeakerAndLeave = frc2::cmd::Sequence(
        PreheatSpeaker(m_speaker).ToPtr(),
        ShootSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(3_s),
        moveForwardsCommand(m_drivetrain)
    );

    m_autoShootTwo = frc2::cmd::Sequence(
        OpenGate(m_gate).ToPtr(),
        PreheatSpeaker(m_speaker).ToPtr(),
        ShootSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(2_s),
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
                PreheatSpeaker(m_speaker).ToPtr().Repeatedly()
            )
        ).WithTimeout(4_s),
        ShootSpeaker(m_intake, m_speaker).ToPtr().WithTimeout(2_s)
    );

    SubsystemRegistry registry{ m_drivetrain, m_intake, m_speaker };

    m_autoPathTest = loadPathFollowCommandFromFile(frc::filesystem::GetDeployDirectory() + "/path_wait.json", registry);
    m_autoBlueSubwoof2nRamp = loadPathFollowCommandFromFile(
        frc::filesystem::GetDeployDirectory() + "/subwoofer-speaker-2n-r-blue.json",
        registry
    );
    m_autoBlueSubwoof3nRampCenter = loadPathFollowCommandFromFile(
        frc::filesystem::GetDeployDirectory() + "/subwoofer-speaker-2n-r-blue.json",
        registry
    );

    m_chooser.SetDefaultOption(auto_::k_None, auto_::k_None);
    m_chooser.AddOption(auto_::k_ShootSpeakerAndStay, auto_::k_ShootSpeakerAndStay);
    m_chooser.AddOption(auto_::k_ShootSpeakerAndLeave, auto_::k_ShootSpeakerAndLeave);
    m_chooser.AddOption(auto_::k_ShootTwo, auto_::k_ShootTwo);
    m_chooser.AddOption(auto_::k_FollowPath, auto_::k_FollowPath);
    m_chooser.AddOption(auto_::k_Subwoof2n, auto_::k_Subwoof2n);
    m_chooser.AddOption(auto_::k_Subwoof3n, auto_::k_Subwoof3n);
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
    } else if (auto_::k_Subwoof2n == m_autoSelected) {
        m_autoBlueSubwoof2nRamp.Schedule();
    } else if (auto_::k_Subwoof3n == m_autoSelected) {
        m_autoBlueSubwoof3nRampCenter.Schedule();
    }

    m_retractAmp.Schedule();
}

void Robot::AutonomousPeriodic() {
    m_drivetrain->UpdateOdometry(); 
}

void Robot::TeleopInit() {
    m_driveTeleopCommand.Schedule();
    m_openGate.Schedule();
    m_retractAmp.Schedule();
}

void Robot::TeleopPeriodic() {
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
        m_shootSpeaker.Schedule();
    } else if (m_operatorController->GetXButtonReleased()) {
        m_shootSpeaker.Cancel();
    }

    if (m_operatorController->GetYButtonPressed()) {
        m_shootSpeakerSlow.Schedule();
    } else if (m_operatorController->GetYButtonReleased()) {
        m_shootSpeakerSlow.Cancel();
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
