#pragma once //makes it so that parent can only be used once

/*******************************
 a parent always has a child a parent in coding also has a
 child example of a parent: animal is the parent of dog
 this would also applie to coding so (line of code)
 is parent of (other line of code)
*******************************/

#include "external/cpptoml.h"

#include "common/WindowAverage.h"
#include "robots/2/commands/Commands.h"

#include <frc/AnalogPotentiometer.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

namespace robot2 {
    using namespace ::robot2;

    class ShooterTiltSubsystem : public frc2::SubsystemBase { //SubsystemBase is the Parent of ShooterTiltSubsystem
        public:
            ShooterTiltSubsystem(std::shared_ptr<cpptoml::table> table);

            void Periodic() override;

            void TiltSpeakerFar();
            void TiltSpeakerNear();
            void TiltAmp();
            void TiltTrap();

            /**
             * @param position Tilt of the shooter.
             */
            void SetTilt(double position, double speed);

            void SetTiltSpeed(double speed);

            void UpdatePosition();
            double GetPosition() const;

            bool IsAboveTopLimit() const;
            bool IsBelowBottomLimit() const;

            void GotoSpeakerPosition();
            void GotoStagePosition();

            double GetSpeakerPosition() const;
            double GetStagePosition() const;

        private:
            rev::CANSparkMax m_tiltMotorLeft;
            rev::CANSparkMax m_tiltMotorRight;

            frc::AnalogPotentiometer m_tiltSensor;
            WindowAverage<8> m_tiltPosition;

            // Config settings loaded from TOML.
            struct {
                double bottomLimit = 0;
                double topLimit = 0;

                double maxSpeed = 0;

                struct {
                    double speaker = 0;
                    double stage = 0;
                } setpoint;
            } m_config;
    };

}
