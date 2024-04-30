#include "robots/1/Interface.h"
#include "robots/1/subsystems/climb/Climb.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

robot1::ClimbSubsystem::ClimbSubsystem(std::shared_ptr<cpptoml::table> table)
:   m_winch(
        interface::climb::k_winchMotor,
        rev::CANSparkMax::MotorType::kBrushless
    ),
    m_winchEncoder(m_winch.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    m_lock(interface::climb::k_lockServo),
    m_limitLeft(interface::climb::k_limitLeft),
    m_limitRight(interface::climb::k_limitRight)
{
    bool hasError = false;

    {
        cpptoml::option<double> lockWinchMicros = table->get_qualified_as<double>("lockWinchMicros");

        if (lockWinchMicros) {
            m_config.lockWinchMicros = units::microsecond_t(*lockWinchMicros);
        } else {
            std::cerr << "Error: climb cannot find toml property climb.lockWinchMicros" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> unlockWinchMicros = table->get_qualified_as<double>("unlockWinchMicros");

        if (unlockWinchMicros) {
            m_config.unlockWinchMicros = units::microsecond_t(*unlockWinchMicros);
        } else {
            std::cerr << "Error: climb cannot find toml property climb.unlockWinchMicros" << std::endl;
            hasError = true;
        }
    }

    {
        cpptoml::option<double> maxSpeed = table->get_qualified_as<double>("maxSpeed");

        if (maxSpeed) {
            m_config.maxSpeed = *maxSpeed;
        } else {
            std::cerr << "Error: climb cannot find toml property climb.maxSpeed" << std::endl;
            hasError = true;
        }
    }

    if (hasError) {
        abort();
    }

    // (+) speed lifts the robot up, pulls arms down.
    m_winch.SetInverted(true);

    m_winchEncoder.SetPosition(0.0);
}

void robot1::ClimbSubsystem::Periodic() { 
    frc::SmartDashboard::PutBoolean("Is Arm Down?", IsArmDown());
    frc::SmartDashboard::PutNumber("Climb Arm Position", GetArmPosition());
}

void robot1::ClimbSubsystem::ClimbUp(double speed) {
    m_winch.Set(speed);
}

void robot1::ClimbSubsystem::ClimbDown(double speed) {
    m_winch.Set(-speed);
}

void robot1::ClimbSubsystem::StopClimb() {
    m_winch.Set(0);
}

double robot1::ClimbSubsystem::GetArmPosition() {
    return m_winchEncoder.GetPosition();
}

bool robot1::ClimbSubsystem::IsArmUp() {
    return false;
}

bool robot1::ClimbSubsystem::IsArmDown() {
    return (!m_limitLeft.Get() || !m_limitRight.Get());
}

bool robot1::ClimbSubsystem::IsArmHome() {
    return false;
}

void robot1::ClimbSubsystem::Lock() {
    m_lock.SetPulseTime(m_config.lockWinchMicros);
    m_isLockEngaged = true;
}

void robot1::ClimbSubsystem::Unlock() {
    m_lock.SetPulseTime(m_config.unlockWinchMicros);
    m_isLockEngaged = false;
}

bool robot1::ClimbSubsystem::IsLockEngaged() {
    return m_isLockEngaged;
}

double robot1::ClimbSubsystem::GetMaxSpeed() {
    return m_config.maxSpeed;
}
