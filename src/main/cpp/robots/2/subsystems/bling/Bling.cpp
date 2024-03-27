#include "robots/2/subsystems/bling/Bling.h"

using namespace ::robot2;

robot2::BlingSubsystem::BlingSubsystem() {}

void robot2::BlingSubsystem::SendCommand(std::string command) {
    static uint8_t end[1] = { '\0' };
}

//MODES
void robot2::BlingSubsystem::BlingOff() {
    SendCommand("m 0");
}

void robot2::BlingSubsystem::BlingRed() {
    SendCommand("m 1");
}

void robot2::BlingSubsystem::BlingYellow() {
    SendCommand("m 2");
}

void robot2::BlingSubsystem::BlingGreen() {
    SendCommand("m 3");
}

void robot2::BlingSubsystem::BlingBlue() {
    SendCommand("m 4");
}

void robot2::BlingSubsystem::BlingOrange() {
    SendCommand("m 5");
}

void robot2::BlingSubsystem::BlingPurple() {
    SendCommand("m 6");
}

void robot2::BlingSubsystem::BlingFlashBlueBlack() {
    SendCommand("m 11");
}


//PERAMITERS
void robot2::BlingSubsystem::BlingEnableAll() {
        SendCommand("p 0");
}

void robot2::BlingSubsystem::BlingEnableClimber1() {
        SendCommand("p 1");
}

void robot2::BlingSubsystem::BlingEnableClimber2() {  
        SendCommand("p 2");
}

void robot2::BlingSubsystem::BlingEnableTrap() {
        SendCommand("p 3");
}

void robot2::BlingSubsystem::BlingEnableSpeaker() {
        SendCommand("p 4");
}

void robot2::BlingSubsystem::BlingLowBrightness() {
        SendCommand("p 11");
}

void robot2::BlingSubsystem::BlingMidBrightness() {
        SendCommand("p 12");
}

void robot2::BlingSubsystem::BlingHighBrightness() {
        SendCommand("p 13");
}

void robot2::BlingSubsystem::BlingSlowFlashRate() {
        SendCommand("p 21");
}

void robot2::BlingSubsystem::BlingMidFlashRate() {
        SendCommand("p 22");
}

void robot2::BlingSubsystem::BlingHighFlashRate() {
        SendCommand("p 23");
}

void robot2::BlingSubsystem::BlingShortTimeout() {
        SendCommand("p 31");
}

void robot2::BlingSubsystem::BlingMidTimeout(){
        SendCommand("p 32");
}

void robot2::BlingSubsystem::BlingHighTimeout(){
        SendCommand("p 33");
}
