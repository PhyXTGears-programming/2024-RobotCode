#include "auto/load/command/Command.h"
#include "auto/load/command/Parallel.h"
#include "auto/load/command/Race.h"
#include "auto/load/command/Sequence.h"

#include <frc2/command/Commands.h>

std::optional<frc2::CommandPtr> importCommand(
    wpi::json & json,
    SubsystemRegistry & registry
) {
    // Command :: { name :: String } | { name :: String, children :: List Command }
    // json :: Command

    auto name = json.value("name", "");

    if ("sequential" == name) {
        return importSequence(json["children"], registry);
    } else if ("parallel" == name) {
        return importParallel(json["children"], registry);
    } else if ("race" == name) {
        return importRace(json["children"], registry);
    } else if ("Wait 5s" == name) {
        return frc2::cmd::Wait(5_s);
    }

    return std::nullopt;
}
