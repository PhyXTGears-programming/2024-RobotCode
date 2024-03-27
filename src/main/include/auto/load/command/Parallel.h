#pragma once

#include "auto/load/SubsystemRegistry.h"

#include <optional>

#include <frc2/command/CommandPtr.h>

#include <wpi/json.h>

std::optional<frc2::CommandPtr> importParallel(wpi::json & json, SubsystemRegistry & registry);
