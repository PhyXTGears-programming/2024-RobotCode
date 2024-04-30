#include "robots/1/commands/ClimbUp.h"
#include "robots/1/subsystems/climb/Climb.h"

using namespace ::robot1;

robot1::ClimbUp::ClimbUp(
    ClimbSubsystem * climb,
    BlingSubsystem * bling,
    frc::XboxController * controller
) {
    AddRequirements(climb);

    m_climb = climb;
    m_bling = bling;
    m_controller = controller;
}

void robot1::ClimbUp::Initialize() {}

void robot1::ClimbUp::Execute() {
    double climbAxis = -m_controller->GetLeftY();

    if (0.2 > std::abs(climbAxis)) {
        m_climb->StopClimb();
    } else if (climbAxis < 0.0 && m_climb->IsLockEngaged()) {
        m_climb->StopClimb();
    } else if (climbAxis > 0.0 && m_climb->IsArmDown()) {
        m_climb->StopClimb();
        m_bling->BlingEnableTrap();
        m_bling->BlingRed();
    } else {
        m_climb->ClimbUp(climbAxis * m_climb->GetMaxSpeed());
    }
}

void robot1::ClimbUp::End(bool interrupted) {
    m_climb->StopClimb();
}

bool robot1::ClimbUp::IsFinished() {
    return false;
}
