#include "auto/load/command/Command.h"
#include "auto/load/command/Sequence.h"

#include <frc2/command/Commands.h>

std::optional<frc2::CommandPtr> importSequence(
    wpi::json & json,
    SubsystemRegistry & registry
) {
    // Command :: { name :: String } | { name :: String, children :: List Command }
    // json :: List Command

    switch (json.size()) {
        case 0:
            return std::nullopt;

        case 1:
            return importCommand(json[0], registry);

        default:
            std::vector<frc2::CommandPtr> commands;

            for (auto cmdJson : json) {
                auto cmd = importCommand(cmdJson, registry);

                if (cmd) {
                    commands.emplace_back(std::move(*cmd));
                }
            }

            return frc2::cmd::Sequence(std::move(commands));
    }
}
