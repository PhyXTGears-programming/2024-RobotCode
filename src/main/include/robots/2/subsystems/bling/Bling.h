#pragma once

#include "robots/2/Interface.h"

#include <frc/DigitalOutput.h>
#include <frc2/command/SubsystemBase.h>

namespace robot2 {
    using namespace ::robot2;

    class BlingSubsystem : public frc2::SubsystemBase {
        public:
            BlingSubsystem();

            void SendCommand(std::string command);

            //MODES
            void BlingOff            ();//turn off bling
            void BlingRed            ();//make bling red
            void BlingYellow         ();//make bling yellow
            void BlingGreen          ();//make bling green
            void BlingBlue           ();//make bling blue
            void BlingOrange         ();//make bling orange
            void BlingPurple         ();//make bling purple
            void BlingFlashBlueBlack ();//flash between blue and black

            //PERAMITERS

            void BlingEnableAll      ();//enable all
            void BlingEnableClimber1 ();//enable theLEFT climber arm
            void BlingEnableClimber2 ();//enable the RIGHT climber arm
            void BlingEnableTrap     ();//enable the trap
            void BlingEnableSpeaker  ();//enable the speaker

            void BlingLowBrightness  ();//set the brightness to low
            void BlingMidBrightness  ();//set the brightness to mid
            void BlingHighBrightness ();//set the brightness to high
            void BlingSlowFlashRate  ();//flash rate is 150 milliseconds between flashes
            void BlingMidFlashRate   ();//flash rate is 250 milliseconds between flashes
            void BlingHighFlashRate  ();//flash rate is 350 milliseconds between flashes
            void BlingShortTimeout   ();//keep LEDs on for 2 seconds
            void BlingMidTimeout     ();//keep LEDs on for 3.5 seconds
            void BlingHighTimeout    ();//keep LEDs on for 5 seconds

            void NotifyNoteAbsent  ();
            void NotifyNotePresent ();

        private:
            frc::DigitalOutput m_noteSignal;
    };

}
