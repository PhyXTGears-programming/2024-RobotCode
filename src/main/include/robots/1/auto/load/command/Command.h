#pragma once

#include "robots/1/auto/load/SubsystemRegistry.h"

#include <optional>

#include <frc2/command/CommandPtr.h>

#include <wpi/json.h>

namespace robot1 {
    std::optional<frc2::CommandPtr> importCommand(wpi::json & son, SubsystemRegistry & registry);
}
