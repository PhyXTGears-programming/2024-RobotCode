#include "robots/1/commands/Cancel.h"

using namespace ::robot1;

robot1::Cancel::Cancel(bool &stopPreheat) : m_signal(stopPreheat) {}

void robot1::Cancel::Initialize() {
    m_signal = true;
}

void robot1::Cancel::Execute() {}

void robot1::Cancel::End(bool interrupted) {}

bool robot1::Cancel::IsFinished() {
    return true;
}
