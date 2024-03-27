#include "robots/1/auto/load/command/Command.h"
#include "robots/1/auto/load/command/Parallel.h"

#include <frc2/command/Commands.h>

using namespace robot1;

std::optional<frc2::CommandPtr> robot1::importParallel(
    wpi::json & json,
    SubsystemRegistry & registry
) {
    // Command :: { name :: String } | { name :: String, children :: List Command }
    // json :: List Command

    const int size = json.size();

    switch (size) {
        case 0:
            return std::nullopt;

        case 1:
            return importCommand(json[0], registry);

        default:
            std::vector<frc2::CommandPtr> commands;

            for (int i = 0; i < size; i += 1) {
                auto cmd = importCommand(json[i], registry);

                if (cmd) {
                    commands.emplace_back(std::move(*cmd));
                }
            }

            return frc2::cmd::Parallel(std::move(commands));
    }
}
