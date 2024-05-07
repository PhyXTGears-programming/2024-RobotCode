#pragma once

#include "external/cpptoml.h"

#include <frc2/command/SubsystemBase.h>

namespace robot2 {
    using namespace ::robot2;

    class LaserSubsystem : public frc2::SubsystemBase {
        public:
            LaserSubsystem(std::shared_ptr<cpptoml::table> table);

            
        private:
            /* data */
    };    
}