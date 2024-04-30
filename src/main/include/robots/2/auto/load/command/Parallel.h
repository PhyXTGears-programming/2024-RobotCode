#pragma once

#include "robots/2/auto/load/SubsystemRegistry.h"

#include <optional>

#include <frc2/command/CommandPtr.h>

#include <wpi/json.h>

namespace robot2 {
    std::optional<frc2::CommandPtr> importParallel(wpi::json & json, SubsystemRegistry & registry);
}
