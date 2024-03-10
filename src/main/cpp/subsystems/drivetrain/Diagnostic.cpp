#include "subsystems/drivetrain/Diagnostic.h"
#include "util/math.h"

#include <cmath>

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/FunctionalCommand.h>

#include <networktables/GenericEntry.h>

#define DASHBOARD_TAB "Diagnostic"

diagnostic::TestDrivetrain::TestDrivetrain(
    Drivetrain * drivetrain,
    frc::XboxController * controller
) {
    m_drivetrain = drivetrain;
    m_controller = controller;
}

// Purpose:
// 1. Measure the turn motor rpm and the wheel turn rpm.
// 2. Calculate the conversion factor from motor_rpm to wheel_rpm.
void diagnostic::TestDrivetrain::Test01MeasureTurnConversionFactor() {
    static int speedFactor = 0;

    static nt::GenericEntry & dashTurnVelocity =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
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

            dashTurnVelocity.SetDouble(
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

// Purpose:
// 1. Measure the rpm of the drive motor and the rpm of the drive wheel.
// 2. Calculate the conversion factor from rpm_motor to rpm_wheel.
// 3. Measure the rpm of the drive wheel vs the voltage of the motor.
// 4. Calculate a best fit line to convert desired wheel speed to motor voltage.
//
// Note: best fit line will be measured with wheels off the ground, so expect
// wheel speeds to drop when robot is placed on the ground and fighting friction
// and inertia.
void diagnostic::TestDrivetrain::Test02MeasureDriveConversionFactor() {
    static int speedFactor = 0;

    static nt::GenericEntry & dashDriveVelocity =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-drive-velocity-rpm", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashDriveVoltage =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-drive-voltage", 0.0)
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

            dashDriveVelocity.SetDouble(
                m_drivetrain->m_frontLeft->m_driveEncoder.GetVelocity()
                * (1.0 / 2.0 * std::numbers::pi)    // rev per radian
                * (60.0 / 1.0)                      // second per minute
            );

            // https://www.chiefdelphi.com/t/get-voltage-from-spark-max/344136/3
            dashDriveVoltage.SetDouble(
                m_drivetrain->m_frontLeft->m_driveMotor.GetAppliedOutput()
                * m_drivetrain->m_frontLeft->m_driveMotor.GetBusVoltage()
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
    // Show relative encoder position.

    static nt::GenericEntry & dashFrontLeftTurnPositionRel =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/FL-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/FL-turn-pos-rel-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashFrontRightTurnPositionRel =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/FR-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/FR-turn-pos-rel-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashBackLeftTurnPositionRel =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/BL-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/BL-turn-pos-rel-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashBackRightTurnPositionRel =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/BR-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/BR-turn-pos-rel-deg", 0.0)
        .GetEntry();

    // Show absolute encoder position.

    static nt::GenericEntry & dashFrontLeftTurnPositionAbs =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/FL-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/FL-turn-pos-abs-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashFrontRightTurnPositionAbs =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/FR-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/FR-turn-pos-abs-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashBackLeftTurnPositionAbs =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/BL-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/BL-turn-pos-abs-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashBackRightTurnPositionAbs =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/BR-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/BR-turn-pos-abs-deg", 0.0)
        .GetEntry();

    // Set absolute encoder zero offset.

    static nt::GenericEntry & dashFrontLeftTurnOffset =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/FL-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/FL-turn-pos-offset-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashFrontRightTurnOffset =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/FR-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/FR-turn-pos-offset-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashBackLeftTurnOffset =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/BL-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/BL-turn-pos-offset-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashBackRightTurnOffset =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .GetLayout("diag/BR-turn-pos-deg", frc::BuiltInLayouts::kList)
        .Add("diag/BR-turn-pos-offset-deg", 0.0)
        .GetEntry();

    // Toggle button to set zero positions.

    static nt::GenericEntry & dashToggleSetZero =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/set-turn-zeroes", false)
        .GetEntry();

    // Toggle button to drive forward at 10%.

    static nt::GenericEntry & dashDriveForward =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/drive-forward", false)
        .GetEntry();

    static double flOffset, frOffset, blOffset, brOffset;

    static frc2::CommandPtr command = frc2::FunctionalCommand(
        [this] () {
            flOffset = m_drivetrain->m_frontLeft->m_absEncoderOffset;
            frOffset = m_drivetrain->m_frontRight->m_absEncoderOffset;
            blOffset = m_drivetrain->m_backLeft->m_absEncoderOffset;
            brOffset = m_drivetrain->m_backRight->m_absEncoderOffset;
        },
        [this] () {
            double offset;

            // Show relative positions.

            dashFrontLeftTurnPositionRel.SetDouble(
                m_drivetrain->m_frontLeft->GetTurnPosition().convert<units::degree>().value()
            );

            dashFrontRightTurnPositionRel.SetDouble(
                m_drivetrain->m_frontRight->GetTurnPosition().convert<units::degree>().value()
            );

            dashBackLeftTurnPositionRel.SetDouble(
                m_drivetrain->m_backLeft->GetTurnPosition().convert<units::degree>().value()
            );

            dashBackRightTurnPositionRel.SetDouble(
                m_drivetrain->m_backRight->GetTurnPosition().convert<units::degree>().value()
            );

            // Show absolute positions.

            dashFrontLeftTurnPositionAbs.SetDouble(
                m_drivetrain->m_frontLeft->GetTurnAbsPosition().convert<units::degree>().value()
            );

            dashFrontRightTurnPositionAbs.SetDouble(
                m_drivetrain->m_frontRight->GetTurnAbsPosition().convert<units::degree>().value()
            );

            dashBackLeftTurnPositionAbs.SetDouble(
                m_drivetrain->m_backLeft->GetTurnAbsPosition().convert<units::degree>().value()
            );

            dashBackRightTurnPositionAbs.SetDouble(
                m_drivetrain->m_backRight->GetTurnAbsPosition().convert<units::degree>().value()
            );

            // Update offsets.

            offset = dashFrontLeftTurnOffset.GetDouble(-999.0);
            if (offset != flOffset) {
                flOffset = offset;
                m_drivetrain->m_frontLeft->m_absEncoderOffset = flOffset;
                m_drivetrain->m_frontLeft->ResetTurnPosition();
            }

            offset = dashFrontRightTurnOffset.GetDouble(-999.0);
            if (offset != frOffset) {
                frOffset = offset;
                m_drivetrain->m_frontRight->m_absEncoderOffset = frOffset;
                m_drivetrain->m_frontRight->ResetTurnPosition();
            }

            offset = dashBackLeftTurnOffset.GetDouble(-999.0);
            if (offset != blOffset) {
                blOffset = offset;
                m_drivetrain->m_backLeft->m_absEncoderOffset = blOffset;
                m_drivetrain->m_backLeft->ResetTurnPosition();
            }

            offset = dashBackRightTurnOffset.GetDouble(-999.0);
            if (offset != brOffset) {
                brOffset = offset;
                m_drivetrain->m_backRight->m_absEncoderOffset = brOffset;
            }

            // Set zeroes.

            if (dashToggleSetZero.GetBoolean(false)) {
                flOffset = -m_drivetrain->m_frontLeft->GetTurnAbsPositionRaw().value();
                m_drivetrain->m_frontLeft->m_absEncoderOffset = flOffset;
                m_drivetrain->m_frontLeft->ResetTurnPosition();

                frOffset = -m_drivetrain->m_frontRight->GetTurnAbsPositionRaw().value();
                m_drivetrain->m_frontRight->m_absEncoderOffset = flOffset;
                m_drivetrain->m_frontRight->ResetTurnPosition();

                blOffset = -m_drivetrain->m_backLeft->GetTurnAbsPositionRaw().value();
                m_drivetrain->m_backLeft->m_absEncoderOffset = flOffset;
                m_drivetrain->m_backLeft->ResetTurnPosition();

                brOffset = -m_drivetrain->m_backRight->GetTurnAbsPositionRaw().value();
                m_drivetrain->m_backRight->m_absEncoderOffset = flOffset;
                m_drivetrain->m_backRight->ResetTurnPosition();

                // Reset toggle to prevent repeat without user interaction.
                dashToggleSetZero.SetBoolean(false);
            }

            // Drive motor

            if (dashDriveForward.GetBoolean(false)) {
                m_drivetrain->m_frontLeft->m_driveMotor.Set(0.1);
                m_drivetrain->m_frontRight->m_driveMotor.Set(0.1);
                m_drivetrain->m_backLeft->m_driveMotor.Set(0.1);
                m_drivetrain->m_backRight->m_driveMotor.Set(0.1);
            } else {
                m_drivetrain->m_frontLeft->m_driveMotor.StopMotor();
                m_drivetrain->m_frontRight->m_driveMotor.StopMotor();
                m_drivetrain->m_backLeft->m_driveMotor.StopMotor();
                m_drivetrain->m_backRight->m_driveMotor.StopMotor();
            }
        },
        [this] (bool interrupted) {
            m_drivetrain->m_frontLeft->m_driveMotor.StopMotor();
            m_drivetrain->m_frontRight->m_driveMotor.StopMotor();
            m_drivetrain->m_backLeft->m_driveMotor.StopMotor();
            m_drivetrain->m_backRight->m_driveMotor.StopMotor();
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
    static double kP = 0.01, kI = 0.0, kD = 0.0;

    // Used to create dashboard widget and retrieve values.
    static frc::PIDController pid(kP, kI, kD);

    frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/tune-turn-pid", pid);

    static nt::GenericEntry & dashTurnPosition =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-turn-position-deg", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashTurnEnable =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-turn-enable", false)
        .GetEntry();

    static frc2::CommandPtr command = frc2::FunctionalCommand(
        [this] () {
            dashTurnEnable.SetBoolean(false);

            rev::SparkPIDController & turnPid =
                m_drivetrain->m_frontLeft->m_turningPid;

            turnPid.SetP(pid.GetP());
            turnPid.SetI(pid.GetI());
            turnPid.SetD(pid.GetD());
            turnPid.SetFF(0.0);
        },
        [this] () {
            if (kP != pid.GetP()) {
                kP = pid.GetP();
                m_drivetrain->m_frontLeft->m_turningPid.SetP(kP);
            }

            if (kI != pid.GetI()) {
                kI = pid.GetI();
                m_drivetrain->m_frontLeft->m_turningPid.SetI(kI);
            }

            if (kD != pid.GetD()) {
                kD = pid.GetD();
                m_drivetrain->m_frontLeft->m_turningPid.SetD(kD);
            }

            if (dashTurnEnable.GetBoolean(false)) {
                m_drivetrain->m_frontLeft->m_turningPid.SetReference(
                    DEG_2_RAD(pid.GetSetpoint()),
                    rev::ControlType::kPosition
                );
            } else {
                m_drivetrain->m_frontLeft->m_turningMotor.StopMotor();
            }

            dashTurnPosition.SetDouble(
                m_drivetrain->m_frontLeft->GetTurnPosition().convert<units::degrees>().value()
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
        "diag/04-tune-turn-pid",
        command.get()
    );
}

void diagnostic::TestDrivetrain::Test05TuneDrivePid() {
    static double kP = 0.01, kI = 0.0, kD = 0.0, kF = 0.0;

    // used to create dashboard widget and retrieve values.
    static frc::PIDController pid(kP, kI, kD);

    frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/tune-drive-pid", pid);

    static nt::GenericEntry & dashDriveFF =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-drive-feed-forward", kF)
        .GetEntry();

    static nt::GenericEntry & dashDriveVelocity =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-drive-velocity-rpm", 0.0)
        .GetEntry();

    static nt::GenericEntry & dashDriveEnable =
        *frc::Shuffleboard::GetTab(DASHBOARD_TAB)
        .Add("diag/front-left-drive-enable", false)
        .GetEntry();

    static frc2::CommandPtr command = frc2::FunctionalCommand(
        [this] () {
            dashDriveEnable.SetBoolean(false);

            rev::SparkPIDController & drivePid =
                m_drivetrain->m_frontLeft->m_drivePid;

            drivePid.SetP(pid.GetP());
            drivePid.SetI(pid.GetI());
            drivePid.SetD(pid.GetD());
            drivePid.SetFF(0.0);    // Use arbFF.
        },
        [this] () {
            if (kP != pid.GetP()) {
                kP = pid.GetP();
                m_drivetrain->m_frontLeft->m_drivePid.SetP(kP);
            }

            if (kI != pid.GetI()) {
                kI = pid.GetI();
                m_drivetrain->m_frontLeft->m_drivePid.SetI(kI);
            }

            if (kD != pid.GetD()) {
                kD = pid.GetD();
                m_drivetrain->m_frontLeft->m_drivePid.SetD(kD);
            }

            if (kF != dashDriveFF.GetDouble(0.0)) {
                kF = dashDriveFF.GetDouble(0.0);
            }

            if (dashDriveEnable.GetBoolean(false)) {
                m_drivetrain->m_frontLeft->m_drivePid.SetReference(
                    pid.GetSetpoint()           // rev per minute
                    * (std::numbers::pi / 1.0)  // radian per rev
                    * (1.0 / 60.0),             // minute per second
                    rev::ControlType::kVelocity,
                    0,
                    std::clamp(kF, -32.0, 32.0)
                );
            }
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
        "diag/05-tune-drive-pid",
        command.get()
    );
}
