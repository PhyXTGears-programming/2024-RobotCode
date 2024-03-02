#pragma once

namespace Interface {
    namespace {
        const int k_can00 = 0;
        const int k_can01 = 1;
        const int k_can02 = 2;
        const int k_can03 = 3;
        const int k_can04 = 4;
        const int k_can05 = 5;
        const int k_can06 = 6;
        const int k_can07 = 7;
        const int k_can08 = 8;

        const int k_can21 = 21;
        const int k_can22 = 22;
        const int k_can23 = 23;
        const int k_can24 = 24;

        const int k_can31 = 31;
        const int k_can32 = 32;
        const int k_can33 = 33;

        const int k_can41 = 41;
        const int k_can42 = 42;

        const int k_can51 = 51;
        const int k_can52 = 52;
        const int k_can53 = 53;

        const int k_can61 = 61;

        const int k_can70 = 70;
        const int k_can71 = 71;

        const int k_dio0 = 0; 

        const int k_pwm0 = 0;
    }

    // Drivetrain

    const int k_drivetrainFrontLeftDrive = k_can04;
    const int k_drivetrainFrontLeftSteer = k_can08;
    const int k_drivetrainFrontLeftEncoder = k_can24;

    const int k_drivetrainFrontRightDrive = k_can02;
    const int k_drivetrainFrontRightSteer = k_can06;
    const int k_drivetrainFrontRightEncoder = k_can22;

    const int k_drivetrainBackLeftDrive = k_can03;
    const int k_drivetrainBackLeftSteer = k_can07;
    const int k_drivetrainBackLeftEncoder = k_can23;

    const int k_drivetrainBackRightDrive = k_can01;
    const int k_drivetrainBackRightSteer = k_can05;
    const int k_drivetrainBackRightEncoder = k_can21;

    // Speaker shooter

    const int k_speakerShooterMotor1 = k_can70;
    const int k_speakerShooterMotor2 = k_can71;

    const int k_speakerShooterNoteSensor = k_dio0;
}
