#pragma once

#include "RobotConfig.h"

#include <frc/I2C.h>

namespace interface {
    namespace {
        // Drivetrain Drive Motors

        const int k_can01 =  1;
        const int k_can02 =  2;
        const int k_can03 =  3;
        const int k_can04 =  4;
        const int k_can05 =  5;
        const int k_can06 =  6;
        const int k_can07 =  7;
        const int k_can08 =  8;

        const int k_can21 = 21;
        const int k_can22 = 22;
        const int k_can23 = 23;
        const int k_can24 = 24;

        // Speaker Shooter

        const int k_can31 = 31;
        const int k_can32 = 32;
        const int k_can33 = 33;
        const int k_can34 = 34;

        // Intake

        const int k_can41 = 41;
        const int k_can42 = 42;

        // Amp

        const int k_can51 = 51;
        const int k_can52 = 52;
        const int k_can53 = 53;

        // Climb

        const int k_can61 = 61;

        const int k_dio0 = 0;
        const int k_dio1 = 1;
        const int k_dio2 = 2;
        const int k_dio3 = 3;

        const int k_dio9 = 9;

        const int k_pwm0 = 0;
        const int k_pwm1 = 1;
        const int k_pwm2 = 2;
    }
}
