#pragma once

namespace Interface {
    namespace {
        const int k_can00 = 0;

        const int k_can41 = 41;
        const int k_can42 = 42;

        const int k_can70 = 70;
        const int k_can71 = 71;

        const int k_dio0 = 0;

        const int k_pwm0 = 0;
    }

    const int k_intakeMotorBottom = k_can41;
    const int k_intakeMotorTop = k_can42;

    const int k_speakerShooterMotor1 = k_can70;
    const int k_speakerShooterMotor2 = k_can71;

    const int k_speakerShooterNoteSensor = k_dio0;
}
