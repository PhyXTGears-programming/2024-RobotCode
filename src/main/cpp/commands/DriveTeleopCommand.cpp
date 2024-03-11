#include "commands/DriveTeleopCommand.h"
#include "subsystems/drivetrain/Drivetrain.h"

#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <networktables/GenericEntry.h>

#include <units/angular_velocity.h>

#define JOYSTICK_DEADZONE 0.2

using units::meters_per_second_t;
using units::radians_per_second_t;

DriveTeleopCommand::DriveTeleopCommand(
    Drivetrain* drivetrain,
    frc::XboxController* driveController,
    units::second_t period
) {
    AddRequirements(drivetrain);

    m_driveController = driveController;
    m_drivetrain = drivetrain;
    m_robotPeriod = period;
}

void DriveTeleopCommand::Initialize() {}

void DriveTeleopCommand::Execute() {
    m_drivetrain->UpdateOdometry();
    //compresses the range of the driving speed to be within the max speed and
    //the minimum. but have the normal speed be the default if no trigger is 
    //being pressed (so both register 0)
    //
    //NOTE no trigger takes priority of the other so if both pressed they will cancel each other 
    double leftTrigger = m_driveController->GetLeftTriggerAxis();   // Range [0.0..1.0]
    double rightTrigger = m_driveController->GetRightTriggerAxis(); // Range [0.0..1.0]

    meters_per_second_t driveReduce = leftTrigger * (constants::k_normalDriveSpeed - constants::k_slowDriveSpeed);
    meters_per_second_t driveGain   = rightTrigger * (constants::k_fastDriveSpeed - constants::k_normalDriveSpeed);

    meters_per_second_t driveSpeedFactor = constants::k_normalDriveSpeed + driveGain - driveReduce;

    radians_per_second_t turnReduce = leftTrigger * (constants::k_maxTurnSpeed - constants::k_slowTurnSpeed);

    radians_per_second_t turnSpeedFactor = constants::k_maxTurnSpeed - turnReduce;

    // the rotation limit is there in case the driver does not want to spin as fast while driving
    const auto forwardSpeed = driveSpeedFactor
        * m_forwardSpeedLimiter.Calculate(frc::ApplyDeadband(
            -m_driveController->GetLeftY(),
            JOYSTICK_DEADZONE)
        );

    const auto strafeSpeed = driveSpeedFactor
        * m_strafeSpeedLimiter.Calculate(frc::ApplyDeadband(
            -m_driveController->GetLeftX(),
            JOYSTICK_DEADZONE)
        );

    const auto turnSpeed = turnSpeedFactor
        * m_turnSpeedLimiter.Calculate(frc::ApplyDeadband(
            m_driveController->GetRightX(),
            JOYSTICK_DEADZONE)
        );

    m_drivetrain->Drive(forwardSpeed, strafeSpeed, turnSpeed, true, m_robotPeriod);
}

void DriveTeleopCommand::End(bool interrupted) {
    // Stop the drive motors!
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s, true, m_robotPeriod);
}

bool DriveTeleopCommand::IsFinished() {
    return false; //dont end because then we wont be able to drive
}
