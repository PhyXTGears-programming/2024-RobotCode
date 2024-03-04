#include "subsystems/drivetrain/Diagnostic.h"
#include "subsystems/drivetrain/Drivetrain.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/XboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/FunctionalCommand.h>

#include <networktables/GenericEntry.h>

#define DASHBOARD_TAB "Diagnostic"

struct diagnostic::TestDrivetrain {
    TestDrivetrain(Drivetrain * drivetrain, frc::XboxController * controller);

    void Test01MeasureTurnConversionFactor();

    void Test02MeasureDriveConversionFactor();

    void Test03MeasureTurnAlignment();

    void Test04TuneTurnPid();

    void Test05TuneDrivePid();

    Drivetrain * m_drivetrain = nullptr;
    frc::XboxController * m_controller = nullptr;
};

diagnostic::TestDrivetrain::TestDrivetrain(
    Drivetrain * drivetrain,
    frc::XboxController * controller
) {
    m_drivetrain = drivetrain;
    m_controller = controller;
}

void diagnostic::TestDrivetrain::Test01MeasureTurnConversionFactor() {
    static int speedFactor = 0;

    static nt::GenericEntry * dashTurnVelocity =
        frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-turn-velocity-rpm", 0.0)
        .GetEntry();

    static frc2::CommandPtr command = frc2::FunctionalCommand(
        [] () { speedFactor = 0; },
        [this] () {
            if (m_controller->GetAButtonPressed()) {
                speedFactor = std::max(0, speedFactor - 1);
            }

            if (m_controller->GetBButtonPressed()) {
                speedFactor = std::min(10, speedFactor + 1);
            }

            if (0 == speedFactor) {
                m_drivetrain->m_frontLeft->m_turningMotor.StopMotor();
            } else {
                m_drivetrain->m_frontLeft->m_turningMotor.Set(speedFactor * 0.10);
            }

            dashTurnVelocity->SetDouble(
                m_drivetrain->m_frontLeft->m_turningEncoder.GetVelocity()
                * (1.0 / 2.0 * std::numbers::pi)    // rev per radian
                * (60.0 / 1.0)                      // second per minute
            );
        },
        [this] (bool interrupted) {
            m_drivetrain->m_frontLeft->m_turningMotor.StopMotor();
        },
        [this] () -> bool {
            return m_controller->GetStartButtonPressed();
        },
        { m_drivetrain }
    ).ToPtr();

    frc::SmartDashboard::PutData(
        "daig/01-measure-turn-conv-factor",
        command.get()
    );
}

void diagnostic::TestDrivetrain::Test02MeasureDriveConversionFactor() {
    static int speedFactor = 0;

    static nt::GenericEntry * dashDriveVelocity =
        frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-drive-velocity-rpm", 0.0)
        .GetEntry();

    static frc2::CommandPtr command = frc2::FunctionalCommand(
        [] () { speedFactor = 0; },
        [this] () {
            if (m_controller->GetAButtonPressed()) {
                speedFactor = std::max(0, speedFactor - 1);
            }

            if (m_controller->GetBButtonPressed()) {
                speedFactor = std::min(10, speedFactor + 1);
            }

            if (0 == speedFactor) {
                m_drivetrain->m_frontLeft->m_driveMotor.StopMotor();
            } else {
                m_drivetrain->m_frontLeft->m_driveMotor.Set(speedFactor * 0.10);
            }

            dashDriveVelocity->SetDouble(
                m_drivetrain->m_frontLeft->m_driveEncoder.GetVelocity()
                * (1.0 / 2.0 * std::numbers::pi)    // rev per radian
                * (60.0 / 1.0)                      // second per minute
            );
        },
        [this] (bool interrupted) {
            m_drivetrain->m_frontLeft->m_driveMotor.StopMotor();
        },
        [this] () -> bool {
            return m_controller->GetStartButtonPressed();
        },
        { m_drivetrain }
    ).ToPtr();

    frc::SmartDashboard::PutData(
        "diag/02-measure-drive-conv-factor",
        command.get()
    );
}

void diagnostic::TestDrivetrain::Test03MeasureTurnAlignment() {
    static nt::GenericEntry * dashTurnPosition =
        frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-turn-position-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry * dashSetTurnPositionDeg =
        frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/set/front-left-turn-position-deg", 0.0)
        .GetEntry();

    static frc2::CommandPtr command = frc2::FunctionalCommand(
        [] () {
            dashSetTurnPositionDeg->SetDouble(0.0);
        },
        [this] () {
            double position = dashSetTurnPositionDeg->GetDouble(0.0);

            m_drivetrain->m_frontLeft->m_turningPid.SetReference(
                position * (std::numbers::pi / 180.0),
                rev::ControlType::kPosition
            );
        },
        [this] (bool interrupted) {
            m_drivetrain->m_frontLeft->m_turningMotor.Set(0.0);
        },
        [this] () -> bool {
            return m_controller->GetStartButtonPressed();
        },
        { m_drivetrain }
    ).ToPtr();

    frc::SmartDashboard::PutData(
        "diag/03-measure-turn-alignment",
        command.get()
    );
}

void diagnostic::TestDrivetrain::Test04TuneTurnPid() {
}

void diagnostic::TestDrivetrain::Test05TuneDrivePid() {
}
