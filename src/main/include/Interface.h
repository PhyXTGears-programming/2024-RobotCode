#pragma once

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

        const int k_pwm0 = 0;
        const int k_pwm1 = 1;
        const int k_pwm2 = 2;
    }

    namespace amp {
        const int k_shootMotor = k_can51;
        const int k_liftMotor  = k_can52;

        const int k_noteSensor     = k_dio1;
        const int k_distanceSensor = k_can53;

        const int k_tiltServo = k_pwm2;
    }

    namespace bling {
        const frc::I2C::Port k_port = frc::I2C::Port::kOnboard;
        const int k_address = 0x30;
    }

    namespace climb {
        const int k_winchMotor = k_can61;

        const int k_lockServo = k_pwm1;

        const int k_limitLeft  = k_dio2;
        const int k_limitRight = k_dio3;
    }

    namespace gate {
        const int k_servo = k_pwm0;
    }

    namespace drive {
        const int k_backRightDrive   = k_can01;
        const int k_backRightTurn    = k_can05;
        const int k_backRightEncoder = k_can21;

        const int k_frontRightDrive   = k_can02;
        const int k_frontRightTurn    = k_can06;
        const int k_frontRightEncoder = k_can22;

        const int k_backLeftDrive   = k_can03;
        const int k_backLeftTurn    = k_can07;
        const int k_backLeftEncoder = k_can23;

        const int k_frontLeftDrive   = k_can04;
        const int k_frontLeftTurn    = k_can08;
        const int k_frontLeftEncoder = k_can24;
    }

    namespace intake {
        const int k_motorBottom = k_can41;
        const int k_motorTop    = k_can42;
    }

    namespace speaker {
        const int k_motor1 = k_can31;
        const int k_motor2 = k_can32;

        const int k_distanceSensor = k_can33;

        const int k_noteSensor = k_dio0;
    }
}
