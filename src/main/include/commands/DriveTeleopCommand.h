#pragma once

#include "subsystems/drivetrain/Drivetrain.h"

#include <units/time.h>

#include <frc/filter/SlewRateLimiter.h>
#include <frc/XboxController.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class DriveTeleopCommand : public frc2::CommandHelper<frc2::Command, DriveTeleopCommand> {
    public:
        DriveTeleopCommand(
            Drivetrain * drivetrain,
            frc::XboxController * driverController,
            units::second_t period = 20_ms
        );

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        units::second_t m_robotPeriod;

        Drivetrain * m_drivetrain;

        frc::XboxController * m_driveController;

        frc::SlewRateLimiter<units::scalar> m_forwardSpeedLimiter{ 3/1_s };
        frc::SlewRateLimiter<units::scalar> m_strafeSpeedLimiter{ 3/1_s };
        frc::SlewRateLimiter<units::scalar> m_turnSpeedLimiter{ 3/1_s };
};
