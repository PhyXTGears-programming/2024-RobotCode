#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

namespace robot1 {

    class Cancel: public frc2::CommandHelper<frc2::Command, Cancel> {
        public:
            Cancel(bool &signal);

            void Initialize() override;
            void Execute() override;
            void End(bool interrupted) override;
            bool IsFinished() override;

        private:
            bool &m_signal;
    };

}
