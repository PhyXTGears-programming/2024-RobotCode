#pragma once

#include <rev/CANSparkMax.h>

struct NoSparkMax {
    NoSparkMax(int, rev::CANSparkMax::MotorType) {};

    void Set(double) {};
    void SetIdleMode(rev::CANSparkMax::IdleMode) {};
    void SetInverted(bool) {};
    void SetSmartCurrentLimit(int) {};

    void StopMotor() {};

    void EnableVoltageCompensation(double) {};

    double GetOutputCurrent() { return 0.0; };
};