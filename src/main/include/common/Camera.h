#pragma once

#include <optional>
#include <string>

#include <cameraserver/CameraServer.h>

namespace camera {
    std::optional<cs::UsbCamera> LoadAndStart(int camId, std::string configFilePath, int resWidth, int resHeight, int fps);
}
