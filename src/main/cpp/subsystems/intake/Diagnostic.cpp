#include "subsystems/intake/Diagnostic.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>

#include <networktables/GenericEntry.h>

#define DIAG_INTAKE_TAB "D-Intake"

diagnostic::TestIntake::TestIntake(
    IntakeSubsystem * intake,
    frc::XboxController * controller
) {
    m_intake = intake;
    m_controller = controller;
}

// Purpose:
// 1. Calibrate direction of bottom intake motor.
// 2. Calibrate direction of top intake motor.
// 3. Calibrate motor speeds.
void diagnostic::TestIntake::Test01TuneIntake() {
    static frc::ShuffleboardLayout & layout =
        frc::Shuffleboard::GetTab(DIAG_INTAKE_TAB)
        .GetLayout("Intake-Test-01", frc::BuiltInLayouts::kList);

    static nt::GenericEntry & dashBottomDirection =
        *layout
        .Add(
            "diag/intake/bottom dir",
            m_intake->m_motorBottom.GetInverted()
        )
        .GetEntry();

    static nt::GenericEntry & dashTopDirection =
        *layout
        .Add(
            "diag/intake/top dir",
            m_intake->m_motorTop.GetInverted()
        )
        .GetEntry();

    static nt::GenericEntry & dashIntakeSpeed =
        *layout
        .Add("diag/intake/set speed %", m_intake->m_config.intakeSpeed)
        .GetEntry();

    {
        static frc2::CommandPtr command = frc2::FunctionalCommand(
            [this] () {
                {
                    double speed = dashIntakeSpeed.GetDouble(999.0);

                    if (999.0 > speed) {
                        m_intake->m_config.intakeSpeed = speed;
                    } else {
                        std::cerr << "Error: TestIntake: cannot read intake speed from dashboard" << std::endl;
                    }
                }

                {
                    bool isInverted = dashBottomDirection.GetBoolean(false);

                    m_intake->m_motorBottom.SetInverted(isInverted);
                }

                {
                    bool isInverted = dashTopDirection.GetBoolean(false);

                    m_intake->m_motorTop.SetInverted(isInverted);
                }
            },
            [this] () {
                m_intake->IntakeAmpShooter();
            },
            [this] (bool interrupted) {
                m_intake->Stop();
            },
            [this] () -> bool {
                return m_controller->GetStartButtonPressed();
            },
            { m_intake }
        ).ToPtr()
            .WithName("Intake Amp");

        frc::SmartDashboard::PutData(
            "diag/intake/01-tune-intake/intake-amp",
            command.get()
        );
    }

    {
        static frc2::CommandPtr command = frc2::FunctionalCommand(
            [this] () {
                {
                    double speed = dashIntakeSpeed.GetDouble(999.0);

                    if (999.0 > speed) {
                        m_intake->m_config.intakeSpeed = speed;
                    } else {
                        std::cerr << "Error: TestIntake: cannot read intake speed from dashboard" << std::endl;
                    }
                }

                {
                    bool isInverted = dashBottomDirection.GetBoolean(false);

                    m_intake->m_motorBottom.SetInverted(isInverted);
                }

                {
                    bool isInverted = dashTopDirection.GetBoolean(false);

                    m_intake->m_motorTop.SetInverted(isInverted);
                }
            },
            [this] () {
                m_intake->IntakeSpeakerShooter();
            },
            [this] (bool interrupted) {
                m_intake->Stop();
            },
            [this] () -> bool {
                return m_controller->GetStartButtonPressed();
            },
            { m_intake }
        ).ToPtr()
            .WithName("Intake Speaker");

        frc::SmartDashboard::PutData(
            "diag/intake/01-tune-intake/intake-speaker",
            command.get()
        );
    }
}
