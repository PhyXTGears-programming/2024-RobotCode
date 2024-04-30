#pragma once

#include "robots/2/subsystems/speaker_shooter/SpeakerShooter.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

using rpm_t = units::revolutions_per_minute_t;

namespace robot2 {

    class PreheatShooter: public frc2::CommandHelper<frc2::Command, PreheatShooter> {
        public:
            PreheatShooter(
                SpeakerShooterSubsystem * speaker,
                rpm_t speed,
                units::volt_t feedForward,
                rpm_t threshold,
                double tilt
            );

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            SpeakerShooterSubsystem * m_speaker = nullptr;

            rpm_t m_speed;
            units::volt_t m_feedForward;
            rpm_t m_threshold;
            double m_tilt;
    };

}
