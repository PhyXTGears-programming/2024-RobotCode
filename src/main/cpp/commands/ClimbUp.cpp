#include "commands/ClimbUp.h"
#include "subsystems/climb/Climb.h"

ClimbUp::ClimbUp(
    ClimbSubsystem * climb,
    frc::XboxController * controller
) {
    AddRequirements(climb);

    m_climb = climb;
    m_controller = controller;
}

void ClimbUp::Initialize() {}

void ClimbUp::Execute() {
    double climbAxis = -m_controller->GetLeftY();

    if (0.2 > std::abs(climbAxis)) {
        m_climb->StopClimb();
    } else {
        m_climb->ClimbUp(climbAxis * m_climb->GetMaxSpeed());
    }
}

void ClimbUp::End(bool interrupted) {
    m_climb->StopClimb();
}

bool ClimbUp::IsFinished() {
    return false;
}