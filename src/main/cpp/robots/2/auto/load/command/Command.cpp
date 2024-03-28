#include "robots/2/auto/load/command/Command.h"
#include "robots/2/auto/load/command/Parallel.h"
#include "robots/2/auto/load/command/Race.h"
#include "robots/2/auto/load/command/Sequence.h"

#include "robots/2/commands/Commands.h"
#include "robots/2/commands/IntakeSpeaker.h"


#include <fmt/core.h>

#include <frc2/command/Commands.h>

using namespace ::robot2;

std::optional<frc2::CommandPtr> robot2::importCommand(
    wpi::json & json,
    SubsystemRegistry & registry
) {
    // Command :: { name :: String } | { name :: String, children :: List Command }
    // json :: Command

    auto name = json.value("name", "");

    if ("sequence" == name) {
        return importSequence(json["children"], registry);
    } else if ("parallel" == name) {
        return importParallel(json["children"], registry);
    } else if ("race" == name) {
        return importRace(json["children"], registry);
    } else if ("Wait 5s" == name) {
        return frc2::cmd::Wait(5_s);
    } else if ("Intake Note" == name) {
        fmt::print("Auto: import command: intake note\n");
        return IntakeSpeaker(registry.intake, registry.speaker).ToPtr();
    } else if ("Preheat Speaker" == name) {
        fmt::print("Auto: import command: preheat shooter\n");
        return cmd::PreheatSpeakerNear(registry.speaker);
    } else if ("Shoot Speaker" == name) {
        fmt::print("Auto: import command: shoot speaker\n");
        return cmd::ShootSpeakerNear(registry.intake, registry.speaker)
            .WithTimeout(1.5_s);
    }

    return std::nullopt;
}
