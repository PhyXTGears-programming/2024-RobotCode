#include "common/Camera.h"

#include <iostream>

#include <cameraserver/CameraServer.h>

#include <wpi/json.h>
#include <wpi/MemoryBuffer.h>

std::optional<cs::UsbCamera> camera::LoadAndStart(int camId, std::string configFilePath, int resWidth, int resHeight, int fps) {
    try {
        std::error_code ec;
        std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
            wpi::MemoryBuffer::GetFile(configFilePath, ec);

        if (nullptr == fileBuffer || ec) {
            std::cerr
                << "Error: Robot: unable to load camera json: "
                << configFilePath
                << std::endl;

            auto camera = frc::CameraServer::StartAutomaticCapture(camId);
            camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
            camera.SetResolution(resWidth, resHeight);
            camera.SetFPS(fps);
            return camera;
        } else {
            wpi::json cameraJson = wpi::json::parse(fileBuffer->begin(), fileBuffer->end());

            auto camera = frc::CameraServer::StartAutomaticCapture(camId);
            camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
            camera.SetConfigJson(cameraJson);
            frc::CameraServer::GetServer().SetSource(camera);
            return camera;
        }
    } catch (...) {
        std::cerr
            << "Error: Robot: unknown exception while configuring camera: "
            << configFilePath
            << std::endl;
        return std::nullopt;
    }
}
