#include "robots/2/subsystems/speaker_shooter/Diagnostic.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>

#include <networktables/GenericEntry.h>

#define DIAG_SPEAKER_TAB "D-Speaker"

using rpm_t = units::revolutions_per_minute_t;

robot2::diagnostic::TestSpeaker::TestSpeaker(SpeakerShooterSubsystem *speaker,
                                             frc::XboxController *controller) {
  m_speaker = speaker;
  m_controller = controller;
}

// Purpose:
// 1. Calibrate shooter flywheel speed.
// 2. Tune Pid
void robot2::diagnostic::TestSpeaker::Test01TuneSpeaker() {
  static double kP = 0.1, kI = 0.0, kD = 0.0, kF = 0.0;

  frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab(DIAG_SPEAKER_TAB);

  static nt::GenericEntry &dashMotorSpeedLeft =
      *tab.Add("diag/speaker/left motor speed rpm",
               m_speaker->m_shootEncoder1.GetVelocity())
           .GetEntry();

  static nt::GenericEntry &dashMotorSpeedRight =
      *tab.Add("diag/speaker/right motor speed rpm",
               m_speaker->m_shootEncoder2.GetVelocity())
           .GetEntry();

  static nt::GenericEntry &dashShootSpeed =
      *tab.Add("diag/speaker/shoot speed rpm",
               m_speaker->m_config.speaker.near.shoot.speed.value())
           .GetEntry();

  static nt::GenericEntry &dashReverseSpeed =
      *tab.Add("diag/speaker/reverse speed rpm",
               m_speaker->m_config.reverseSpeed.value())
           .GetEntry();

  static nt::GenericEntry &dashP = *tab.Add("diag/speaker/kP", kP).GetEntry();

  static nt::GenericEntry &dashI = *tab.Add("diag/speaker/kI", kI).GetEntry();

  static nt::GenericEntry &dashD = *tab.Add("diag/speaker/kD", kD).GetEntry();

  static nt::GenericEntry &dashF = *tab.Add("diag/speaker/kF", kF).GetEntry();

  {
    static frc2::CommandPtr command =
        frc2::FunctionalCommand(
            []() {},
            [this]() {
              dashMotorSpeedLeft.SetDouble(
                  m_speaker->m_shootEncoder1.GetVelocity());
              dashMotorSpeedRight.SetDouble(
                  m_speaker->m_shootEncoder2.GetVelocity());
            },
            [](bool interrupted) {},
            [this]() -> bool { return m_controller->GetStartButtonPressed(); },
            {})
            .ToPtr()
            .WithName("Report Speeds");

    frc::SmartDashboard::PutData("diag/speaker/01-tune-speaker/report-speeds",
                                 command.get());
  }

  auto readPidFromDash = []() {
    kP = dashP.GetDouble(0.0);
    kI = dashI.GetDouble(0.0);
    kD = dashD.GetDouble(0.0);
    kF = dashF.GetDouble(0.0);
  };

  {
    static rpm_t &speed = *new rpm_t(0.0);
    static units::volt_t &feedForward = *new units::volt_t(0.0);

    static frc2::CommandPtr command =
        frc2::FunctionalCommand(
            [this, &readPidFromDash]() {
              {
                double s = dashShootSpeed.GetDouble(-1.0);

                if (0.0 <= s) {
                  speed = rpm_t(s);
                } else {
                  speed = 0.0_rpm;
                  std::cerr << "Error: TestSpeaker: cannot read shoot speed "
                               "from dashboard"
                            << std::endl;
                }
              }

              readPidFromDash();

              m_speaker->m_shootPid1.SetP(kP);
              m_speaker->m_shootPid1.SetI(kI);
              m_speaker->m_shootPid1.SetD(kD);
              feedForward = units::volt_t(kF);
            },
            [this]() { m_speaker->SetShooterSpeed(speed, feedForward); },
            [this](bool interrupted) { m_speaker->StopShooter(); },
            [this]() -> bool { return m_controller->GetStartButtonPressed(); },
            {m_speaker})
            .ToPtr()
            .WithName("Shoot Fast");

    frc::SmartDashboard::PutData("diag/speaker/01-tune-speaker/shoot",
                                 command.get());
  }

  {
    static rpm_t &speed = *new rpm_t(0.0);
    static units::volt_t &feedForward = *new units::volt_t(0.0);

    static frc2::CommandPtr command =
        frc2::FunctionalCommand(
            [this, &readPidFromDash]() {
              {
                double s = dashReverseSpeed.GetDouble(-1.0);

                if (0.0 <= s) {
                  speed = rpm_t(s);
                } else {
                  speed = 0.0_rpm;
                  std::cerr << "Error: TestSpeaker: cannot read reverse speed "
                               "from dashboard"
                            << std::endl;
                }
              }

              readPidFromDash();

              m_speaker->m_shootPid1.SetP(kP);
              m_speaker->m_shootPid1.SetI(kI);
              m_speaker->m_shootPid1.SetD(kD);

              feedForward = units::volt_t(kF);
            },
            [this]() { m_speaker->SetShooterSpeed(-speed, -feedForward); },
            [this](bool interrupted) { m_speaker->StopShooter(); },
            [this]() -> bool { return m_controller->GetStartButtonPressed(); },
            {m_speaker})
            .ToPtr()
            .WithName("Reverse");

    frc::SmartDashboard::PutData("diag/speaker/01-tune-speaker/reverse",
                                 command.get());
  }
}
