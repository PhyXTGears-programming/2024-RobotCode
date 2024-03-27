// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotConfig.h"

#if ROBOT_ROBACH == ROBOT_ID
    #include "robots/1/Robot.h"
    using namespace robot1;
#elif ROBOT_BOTTHOVEN == ROBOT_ID
#endif

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
