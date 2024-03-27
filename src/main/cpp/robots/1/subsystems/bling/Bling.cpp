#include "robots/1/subsystems/bling/Bling.h"

robot1::BlingSubsystem::BlingSubsystem() {


}

void robot1::BlingSubsystem::SendCommand(std::string command) {
    static uint8_t end[1] = { '\0' };

    m_pico.WriteBulk(
        reinterpret_cast<uint8_t *>(
            const_cast<char *>(command.c_str())
        ),
        command.length()
    );

    m_pico.WriteBulk(end, 1);
}

//MODES
void robot1::BlingSubsystem::BlingOff() {
    SendCommand("m 0");
}

void robot1::BlingSubsystem::BlingRed() {
    SendCommand("m 1");
}

void robot1::BlingSubsystem::BlingYellow() {
    SendCommand("m 2");
}

void robot1::BlingSubsystem::BlingGreen() {
    SendCommand("m 3");
}

void robot1::BlingSubsystem::BlingBlue() {
    SendCommand("m 4");
}

void robot1::BlingSubsystem::BlingOrange() {
    SendCommand("m 5");
}

void robot1::BlingSubsystem::BlingPurple() {
    SendCommand("m 6");
}

void robot1::BlingSubsystem::BlingFlashBlueBlack() {
    SendCommand("m 11");
}


//PERAMITERS
void robot1::BlingSubsystem::BlingEnableAll() {
        SendCommand("p 0");
}

void robot1::BlingSubsystem::BlingEnableClimber1() {
        SendCommand("p 1");
}

void robot1::BlingSubsystem::BlingEnableClimber2() {  
        SendCommand("p 2");
}

void robot1::BlingSubsystem::BlingEnableTrap() {
        SendCommand("p 3");
}

void robot1::BlingSubsystem::BlingEnableSpeaker() {
        SendCommand("p 4");
}

void robot1::BlingSubsystem::BlingLowBrightness() {
        SendCommand("p 11");
}

void robot1::BlingSubsystem::BlingMidBrightness() {
        SendCommand("p 12");
}

void robot1::BlingSubsystem::BlingHighBrightness() {
        SendCommand("p 13");
}

void robot1::BlingSubsystem::BlingSlowFlashRate() {
        SendCommand("p 21");
}

void robot1::BlingSubsystem::BlingMidFlashRate() {
        SendCommand("p 22");
}

void robot1::BlingSubsystem::BlingHighFlashRate() {
        SendCommand("p 23");
}

void robot1::BlingSubsystem::BlingShortTimeout() {
        SendCommand("p 31");
}

void robot1::BlingSubsystem::BlingMidTimeout(){
        SendCommand("p 32");
}

void robot1::BlingSubsystem::BlingHighTimeout(){
        SendCommand("p 33");
}
