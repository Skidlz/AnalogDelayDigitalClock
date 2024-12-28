#include <stdio.h>
#pragma once
class MIDI {
public:
    uint8_t chan = 0xff;

    void handleByte(uint8_t currentByte);

    void (*noteOff)(uint8_t note, uint8_t vel);
    void (*noteOn)(uint8_t note, uint8_t vel);
    void (*controlChange)(uint8_t cc, uint8_t val);
    void (*prgChange)(uint8_t);
    void (*pitchBend)(int16_t);
    void (*songPosPtr)(uint16_t);
    void (*rtClock)();
    void (*rtStart)();
    void (*rtCont)();
    void (*rtStop)();

private:
    enum message {
        //CHANNEL VOICE MESSAGES
        NOTE_OFF = 0x80, //Note Off
        NOTE_ON = 0x90, //Note On
        AFT_TCH = 0xA0, //Polyphonic Key Pressure (Aftertouch)
        CC = 0xB0, //Control Change
        PRG_CHNG = 0xC0, //Program Change
        CHN_AT = 0xD0, //Channel Pressure (Aftertouch)
        BEND = 0xE0, //Pitch Bend Change
        //SYSTEM COMMON MESSAGES
        SYSEX = 0xF0,
        QRT_FRAME = 0xF1, //MIDI Time Code Quarter Frame
        POS_PNT = 0xF2, //Song Position Pointer
        SONG_SEL = 0xF3, //Song Select
        TUNE = 0xF6, //Tune Request
        EOX = 0xF7, //EOX: "End of System Exclusive" flag
        //SYSTEM REAL TIME MESSAGES
        CLOCK = 0xF8, //Timing Clock
        START = 0xFA, //Start
        CONT = 0xFB, //Continue
        STOP = 0xFC, //Stop
        SENS = 0xFE, //Active Sensing
        RESET = 0xFF //System Reset
    };

    uint8_t handleChanMess();
    uint8_t skipMessage();
    uint8_t handleSysComMess();
    void handleRealTime(uint8_t mess);
    uint8_t lastStatusByte;
    uint8_t byteCount; //count of received data bytes
    uint8_t dataBytes[2];
};