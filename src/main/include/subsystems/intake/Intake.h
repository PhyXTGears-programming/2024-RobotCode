#pragma once

#include "external/cpptoml.h"
#include "subsystems/intake/DiagnosticDecl.h"

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(std::shared_ptr<cpptoml::table> table);

        void IntakeAmpShooter();

        void IntakeSpeakerShooter();

        void ReverseAmpShooter();

        void ReverseSpeakerShooter();

        void Stop();

    private:
        rev::CANSparkMax m_motorBottom;
        rev::CANSparkMax m_motorTop;

        struct {
            double intakeSpeed;

        } m_config;

    friend class diagnostic::TestIntake;
};
