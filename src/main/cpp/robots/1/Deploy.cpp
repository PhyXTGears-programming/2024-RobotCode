#include "robots/1/Deploy.h"

#include <frc/Filesystem.h>

std::string robot1::deploy::GetRobotDirectory() {
    return frc::filesystem::GetDeployDirectory() + "/robots/1";
}
