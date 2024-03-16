#include "subsystems/bling/Bling.h"
#include <frc2/command/SubsystemBase.h>

BlingSubsystem::BlingSubsystem() {


}

void BlingSubsystem::SendCommand(std::string command) {
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
void BlingSubsystem::BlingOff() {
    SendCommand("m 0");
}

void BlingSubsystem::BlingRed() {
    SendCommand("m 1");
}

void BlingSubsystem::BlingYellow() {
    SendCommand("m 2");
}

void BlingSubsystem::BlingGreen() {
    SendCommand("m 3");
}

void BlingSubsystem::BlingBlue() {
    SendCommand("m 4");
}

void BlingSubsystem::BlingOrange() {
    SendCommand("m 5");
}

void BlingSubsystem::BlingPurple() {
    SendCommand("m 6");
}

void BlingSubsystem::BlingFlashBlueBlack() {
    SendCommand("m 11");
}


//PERAMITERS
void BlingSubsystem::BlingEnableClimber1() {
        SendCommand("p 1");
}

void BlingSubsystem::BlingEnableClimber2() {  
        SendCommand("p 2");
}

void BlingSubsystem::BlingEnableTrap() {
        SendCommand("p 3");
}

void BlingSubsystem::BlingEnableSpeaker() {
        SendCommand("p 4");
}

void BlingSubsystem::BlingLowBrightness() {
        SendCommand("p 11");
}

void BlingSubsystem::BlingMidBrightness() {
        SendCommand("p 12");
}

void BlingSubsystem::BlingHighBrightness() {
        SendCommand("p 13");
}

void BlingSubsystem::BlingSlowFlashRate() {
        SendCommand("p 21");
}

void BlingSubsystem::BlingMidFlashRate() {
        SendCommand("p 22");
}

void BlingSubsystem::BlingHighFlashRate() {
        SendCommand("p 23");
}

void BlingSubsystem::BlingShortTimeout() {
        SendCommand("p 31");
}

void BlingSubsystem::BlingMidTimeout(){
        SendCommand("p 32");
}

void BlingSubsystem::BlingHighTimeout(){
        SendCommand("p 33");
}