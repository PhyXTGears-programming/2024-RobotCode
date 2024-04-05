#pragma once

class NoSparkRelativeEncoder {
    public:
        NoSparkRelativeEncoder(NoSparkRelativeEncoder &&) = default;
        NoSparkRelativeEncoder(NoSparkRelativeEncoder &) = default;
        NoSparkRelativeEncoder() = default;

        void SetPosition(float) {};
        float GetPosition() { return 0.0; };
        
        void SetVelocity(float) {};
        float GetVelocity() { return 0.0; };
};