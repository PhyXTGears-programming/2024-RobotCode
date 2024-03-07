#pragma once

namespace Interface {
    namespace {
        const int k_can00 = 0;

        const int k_can61 = 61;
        const int k_can62 = 62;

        const int k_can70 = 70;
        const int k_can71 = 71;

        const int k_dio0 = 0;
        const int k_dio1 = 1;

        const int k_pwm0 = 0;
    }

    const int k_ampShooterLiftMotor = k_can62;
    const int k_ampShooterShootMotor = k_can61;
    const int k_ampShooterServo = k_pwm0;
    const int k_ampShooterNoteSensor = k_dio1;

    const int k_speakerShooterMotor1 = k_can70;
    const int k_speakerShooterMotor2 = k_can71;

    const int k_speakerShooterNoteSensor = k_dio0;
}
