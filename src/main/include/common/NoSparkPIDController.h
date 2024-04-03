#pragma once

#include <rev/CANSparkMax.h>
#include "common/NoSparkRelativeEncoder.h"

class NoSparkPIDController {
    public:
        NoSparkPIDController() = default;
        NoSparkPIDController(NoSparkPIDController &&) = default;

        void SetP(double) {};
        void SetI(double) {};
        void SetD(double) {};

        void SetFeedbackDevice(NoSparkRelativeEncoder) {};

        void SetReference(double, rev::ControlType, int, double) {};
};