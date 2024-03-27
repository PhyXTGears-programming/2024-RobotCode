#pragma once

// Use a forward declaration to create an incomplete type, so that we may use
// `friend class diagnostic::TestDrivetrain` without cyclic dependencies.

namespace robot1::diagnostic {
    class TestDrivetrain;
}
