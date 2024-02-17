#pragma once

#include "subsystems/climb/Climb.h"

#include <frc2/command/SubsystemBase.h>

ClimbSubsystem::ClimbSubsystem() {}

void ClimbSubsystem::ClimbUp(double speed) {}

void ClimbSubsystem::ClimbDown(double speed) {}

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

void ClimbSubsystem::Lock() {}

void ClimbSubsystem::Unlock() {}

bool ClimbSubsystem::IsLockEngaged() {
    return false;
}