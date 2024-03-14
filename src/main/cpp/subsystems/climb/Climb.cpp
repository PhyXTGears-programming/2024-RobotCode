#pragma once

#include "Interface.h"
#include "subsystems/climb/Climb.h"

#include <frc2/command/SubsystemBase.h>

ClimbSubsystem::ClimbSubsystem()
:   m_winch(
        interface::climb::k_winchMotor,
        rev::CANSparkMax::MotorType::kBrushless
    ),
    m_lock(interface::climb::k_lockServo)
{
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