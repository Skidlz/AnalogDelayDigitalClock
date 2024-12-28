#pragma once
#include <stdint.h>

namespace AD9833 {
    //AD9833 http://www.vwlowen.co.uk/arduino/AD9833-waveform-generator/AD9833-waveform-generator.htm
    const uint8_t FSYNC_PIN = 10; //chip select pin
    const uint8_t SQUARE = 0x28;
    const uint8_t FREQ0 = 14;
    const uint8_t RESET = 8;

    void init();
    void write(uint16_t dat);
    void setFreq(uint32_t frequency);
}