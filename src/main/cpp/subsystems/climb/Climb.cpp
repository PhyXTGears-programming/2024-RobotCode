#pragma once

#include "Interface.h"
#include "subsystems/climb/Climb.h"

#include <iostream>

#include <frc2/command/SubsystemBase.h>

ClimbSubsystem::ClimbSubsystem(std::shared_ptr<cpptoml::table> table)
:   m_winch(
        interface::climb::k_winchMotor,
        rev::CANSparkMax::MotorType::kBrushless
    ),
    m_lock(interface::climb::k_lockServo)
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
}

void ClimbSubsystem::ClimbUp(double speed) {
    SetClimbSpeed(std::max(0.0, speed));
}

void ClimbSubsystem::ClimbDown(double speed) {
    SetClimbSpeed(std::min(0.0, speed));
}

void ClimbSubsystem::SetClimbSpeed(double speed) {
    m_winch.Set(speed);
}

void ClimbSubsystem::StopClimb() {
    m_winch.Set(0);
}

double ClimbSubsystem::GetArmPosition() {
    return 0.0;
}

bool ClimbSubsystem::IsArmUp() {
    return false;
}

bool ClimbSubsystem::IsArmDown() {
    return false;
}

bool ClimbSubsystem::IsArmHome() {
    return false;
}

void ClimbSubsystem::Lock() {
    m_lock.SetPulseTime(m_config.lockWinchMicros);
    m_isLockEngaged = true;
}

void ClimbSubsystem::Unlock() {
    m_lock.SetPulseTime(m_config.unlockWinchMicros);
    m_isLockEngaged = false;
}

bool ClimbSubsystem::IsLockEngaged() {
    return m_isLockEngaged;
}