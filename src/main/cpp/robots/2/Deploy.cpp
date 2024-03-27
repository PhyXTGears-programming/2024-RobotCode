#include "robots/2/Deploy.h"

#include <frc/Filesystem.h>

std::string robot2::deploy::GetRobotDirectory() {
    return frc::filesystem::GetDeployDirectory() + "/robots/2";
}
