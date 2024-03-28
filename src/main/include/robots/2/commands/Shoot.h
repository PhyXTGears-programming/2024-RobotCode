#pragma once

#include "robots/2/subsystems/intake/Intake.h"
#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

using rpm_t = units::revolutions_per_minute_t;

namespace robot2 {

    class Shoot : public frc2::CommandHelper<frc2::Command, Shoot> {
        public:
            Shoot(
                IntakeSubsystem * intake,
                SpeakerShooterSubsystem * speaker,
                rpm_t speed,
                units::volt_t feedForward,
                units::microsecond_t leftMicros,
                units::microsecond_t rightMicros
            );

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            IntakeSubsystem * m_intake = nullptr;
            SpeakerShooterSubsystem * m_speaker = nullptr;

            rpm_t m_speed;
            units::volt_t m_feedForward;
            units::microsecond_t m_leftMicros;
            units::microsecond_t m_rightMicros;
    };

}
