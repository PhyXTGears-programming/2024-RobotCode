#pragma once

#include "common/NoSparkRelativeEncoder.h"
#include "common/NoSparkPIDController.h"

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkRelativeEncoder.h>

struct NoSparkMax {
    public:
        NoSparkMax(int, rev::CANSparkMax::MotorType) {};
        NoSparkMax(int, rev::CANSparkMaxLowLevel::MotorType) {};

        void Set(double) {};
        void SetIdleMode(rev::CANSparkMax::IdleMode) {};
        void SetInverted(bool) {};
        void SetSmartCurrentLimit(int) {};

        void StopMotor() {};

        void EnableVoltageCompensation(double) {};

        double GetOutputCurrent() { return 0.0; };

        bool GetInverted() { return false; };

        void Follow(NoSparkMax &, bool) {};

        NoSparkRelativeEncoder GetEncoder(rev::SparkRelativeEncoder::Type) { return std::move(NoSparkRelativeEncoder()); };

        NoSparkPIDController GetPIDController() { return NoSparkPIDController(); };
};