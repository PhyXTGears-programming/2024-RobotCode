#include "robots/1/Interface.h"
#include "robots/1/subsystems/intake/Intake.h"

#include <iostream>

#include <frc2/command/SubsystemBase.h>

robot1::IntakeSubsystem::IntakeSubsystem(std::shared_ptr<cpptoml::table> table)
:
    m_motorBottom(interface::intake::k_motorBottom, rev::CANSparkMax::MotorType::kBrushless),
    m_motorTop(interface::intake::k_motorTop, rev::CANSparkMax::MotorType::kBrushless)
{
    bool hasError = false;

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("intakeSpeed");

        if (speed) {
            m_config.intakeSpeed = *speed;
        } else {
            std::cerr << "Error: intake cannot find toml intake.intakeSpeed" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("reverseSpeed");

        if (speed) {
            m_config.reverseSpeed = *speed;
        } else {
            std::cerr << "Error: intake cannot find toml intake.reverseSpeed" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    m_motorBottom.SetInverted(true);
}

void robot1::IntakeSubsystem::IntakeAmpShooter() {
    m_motorTop.Set(m_config.intakeSpeed);
    m_motorBottom.Set(m_config.intakeSpeed);
}

void robot1::IntakeSubsystem::IntakeSpeakerShooter() {
    m_motorTop.Set(-m_config.intakeSpeed);
    m_motorBottom.Set(m_config.intakeSpeed);
}

void robot1::IntakeSubsystem::ReverseAmpShooter() {
    m_motorTop.Set(-m_config.reverseSpeed);
    m_motorBottom.Set(-m_config.reverseSpeed);
}

void robot1::IntakeSubsystem::ReverseSpeakerShooter() {
    m_motorTop.Set(m_config.reverseSpeed);
    m_motorBottom.Set(-m_config.reverseSpeed);
}

void robot1::IntakeSubsystem::Stop() {
    m_motorTop.StopMotor();
    m_motorBottom.StopMotor();
}
