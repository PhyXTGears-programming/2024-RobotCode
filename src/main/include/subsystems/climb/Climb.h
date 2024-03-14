#pragma once

#include "external/cpptoml.h"

#include <frc/Servo.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

class ClimbSubsystem : public frc2::SubsystemBase {
    public:
        ClimbSubsystem(std::shared_ptr<cpptoml::table> table);

        // Tells arm to go up | speed is a placeholder
        void ClimbUp(double speed);

        // Tells arm to go down
        void ClimbDown(double speed);

        void SetClimbSpeed(double speed);

        void StopClimb();

        // Gets the arm position
        double GetArmPosition();

        // Asks arm if it is up
        bool IsArmUp();

        // Asks arm if it is down
        bool IsArmDown();

        // Asks arm if it can go under stage (halfway down)
        bool IsArmHome();

        // Locks winch
        void Lock();
        
        // Unlocks winch
        void Unlock();
        
        //asks the status of the lock
        bool IsLockEngaged();

    private:
        rev::CANSparkMax m_winch;

        frc::Servo m_lock;

        bool m_isLockEngaged = true;

        struct {
            double maxSpeed;
            units::microsecond_t lockWinchMicros = 0.0_us;
            units::microsecond_t unlockWinchMicros = 0.0_us;
        } m_config;
};
