#pragma once

#include "common/InterfacePorts.h"

namespace robot2::interface {
    using namespace ::interface;

    const int k_robotId = k_dio9;

    namespace bling {
        const int k_noteSignal = k_dio4;
    }

    namespace climb {
        const int k_winchMotor = k_can61;

        const int k_lockServo = k_pwm1;

        const int k_limitLeft  = k_dio2;
        const int k_limitRight = k_dio3;
    }

    namespace gate {
        const int k_servo = k_pwm0;
    }

    namespace drive {
        const int k_backRightDrive   = k_can01;
        const int k_backRightTurn    = k_can05;
        const int k_backRightEncoder = k_can21;

        const int k_frontRightDrive   = k_can02;
        const int k_frontRightTurn    = k_can06;
        const int k_frontRightEncoder = k_can22;

        const int k_backLeftDrive   = k_can03;
        const int k_backLeftTurn    = k_can07;
        const int k_backLeftEncoder = k_can23;

        const int k_frontLeftDrive   = k_can04;
        const int k_frontLeftTurn    = k_can08;
        const int k_frontLeftEncoder = k_can24;
    }

    namespace intake {
        const int k_motorBottom = k_can41;
        const int k_motorTop    = k_can42;
    }

    namespace speaker {
        const int k_motor1 = k_can31;
        const int k_motor2 = k_can32;

        const int k_feedMotor = k_can33;

        const int k_distanceSensorLeft = k_can34;
        const int k_distanceSensorRight = k_can33;

        const int k_noteSensorBottom = k_dio0;
        const int k_noteSensorTop    = k_dio1;

        const int k_tiltLeft = k_pwm2;
        const int k_tiltRight = k_pwm3;
    }

    namespace tilt {
        const int k_tiltLeft = k_can36;
        const int k_tiltRight = k_can37;

        const int k_tiltSensor = k_ai1;
    }
}
