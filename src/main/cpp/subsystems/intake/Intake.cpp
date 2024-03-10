#include "Interface.h"
#include "subsystems/intake/Intake.h"


#include <frc2/command/SubsystemBase.h>



IntakeSubsystem::IntakeSubsystem(std::shared_ptr<cpptoml::table> table)
    : m_motorBottom(Interface::intake::k_motorBottom, rev::CANSparkMax::MotorType::kBrushless), 
    m_motorTop(Interface::intake::k_motorTop, rev::CANSparkMax::MotorType::kBrushless)
{
    {
        cpptoml::option<double> speed = table->get_qualified_as<double>("intakeSpeed");

        if (!speed) {
            throw "Error: intake cannot find toml intake.intakeSpeed";
        }
        
        m_config.intakeSpeed = *speed;
    }

    m_motorBottom.SetInverted(true);
}

void IntakeSubsystem::IntakeAmpShooter() {
    m_motorTop.Set(m_config.intakeSpeed);
    m_motorBottom.Set(m_config.intakeSpeed);
}

void IntakeSubsystem::IntakeSpeakerShooter() {
    m_motorTop.Set(-m_config.intakeSpeed);
    m_motorBottom.Set(m_config.intakeSpeed);
}

void IntakeSubsystem::ReverseAmpShooter() {
    m_motorTop.Set(-m_config.intakeSpeed);
    m_motorBottom.Set(-m_config.intakeSpeed);
}

void IntakeSubsystem::ReverseSpeakerShooter() {
    m_motorTop.Set(m_config.intakeSpeed);
    m_motorBottom.Set(-m_config.intakeSpeed);
}

void IntakeSubsystem::Stop() {
    m_motorTop.StopMotor();
    m_motorBottom.StopMotor();
}
