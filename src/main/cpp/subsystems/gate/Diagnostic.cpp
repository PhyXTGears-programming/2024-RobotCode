#include "subsystems/gate/Diagnostic.h"

#include <iostream>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/FunctionalCommand.h>

#include <networktables/GenericEntry.h>

#define DIAG_GATE_TAB "D-Gate"

diagnostic::TestGate::TestGate(
    GateSubsystem * gate,
    frc::XboxController * controller
) {
    m_gate = gate;
    m_controller = controller;
}

// Purpose:
// 1. Load config setpoints for testing.
// 2. Test alternative setpoints.
void diagnostic::TestGate::Test01TuneServo() {
    static frc::ShuffleboardLayout & layout =
        frc::Shuffleboard::GetTab(DIAG_GATE_TAB)
        .GetLayout("Gate-Test-01", frc::BuiltInLayouts::kList);

    static nt::GenericEntry & dashOpenSetpoint =
        *layout
        .Add("diag/open-us", m_gate->m_config.openMicros.value())
        .GetEntry();

    static nt::GenericEntry & dashCloseSetpoint =
        *layout
        .Add("diag/close-us", m_gate->m_config.closeMicros.value())
        .GetEntry();

    {
        static frc2::CommandPtr command = frc2::FunctionalCommand(
            [] () {},
            [this] () {
                const double setpoint = dashOpenSetpoint.GetDouble(0.0);

                if (0.0 < setpoint) {
                    m_gate->m_servo.SetPulseTime(units::microsecond_t(
                        setpoint
                    ));
                } else {
                    std::cerr << "Error: TestGate: cannot read open setpoint from dashboard" << std::endl;
                }
            },
            [] (bool interrupted) {},
            [this] () -> bool {
                return m_controller->GetStartButtonPressed();
            },
            { m_gate }
        ).ToPtr()
            .WithName("Open Gate");

        frc::SmartDashboard::PutData(
            "diag/gate/01-tune-servo/open-gate",
            command.get()
        );
    }

    {
        static frc2::CommandPtr command = frc2::FunctionalCommand(
            [] () {},
            [this] () {
                const double setpoint = dashCloseSetpoint.GetDouble(0.0);

                if (0.0 < setpoint) {
                    m_gate->m_servo.SetPulseTime(units::microsecond_t(
                        setpoint
                    ));
                } else {
                    std::cerr << "Error: TestGate: cannot read close setpoint from dashboard" << std::endl;
                }
            },
            [] (bool interrupted) {},
            [this] () -> bool {
                return m_controller->GetStartButtonPressed();
            },
            { m_gate }
        ).WithName("Close Gate");

        static auto& _dashCommand =
            *layout
            .Add("diag/gate/01-tune-servo/close-gate", command.get())
            .WithWidget(frc::BuiltInWidgets::kCommand)
            .GetEntry();
    }

    {
        static frc2::CommandPtr command = frc2::FunctionalCommand(
            [this] () {
                dashOpenSetpoint.SetDouble(m_gate->m_config.openMicros.value());
                dashCloseSetpoint.SetDouble(m_gate->m_config.closeMicros.value());
            },
            [] () {},
            [] (bool interrupted) {},
            [] () -> bool { return true; },
            { m_gate }
        ).ToPtr()
            .WithName("Reload Setpoints");

        frc::SmartDashboard::PutData(
            "diag/gate/01-tune-servo/reload-setpoints",
            command.get()
        );
    }
}
