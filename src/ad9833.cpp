#include "ad9833.h"
#include <Arduino.h>
#include <SPI.h>

//AD9833 http://www.vwlowen.co.uk/arduino/AD9833-waveform-generator/AD9833-waveform-generator.htm
#define FSYNC 10 //chip select pin
#define SINE  0x00
#define SQUARE 0x28
#define TRIANGLE 0x02

void initAD() {
    SPI.begin();
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE2));
    delay(50);
    writeADreg(1 << 8); //write AD9833 reset bit
    delay(50);
    writeADreg((1 << 13) | SQUARE); //B28 set so we can load FREQx in two writes

    setADfreq(10000); //10khz
}

void setADfreq(uint32_t frequency) { //set AD9833 frequency
    //multiply by 40 because we divide by 40 for filter
    uint32_t freqWord = frequency * 40 * (268435456.0 / 25000000.0); //2^28 / crystal freq
    //only lower 14 bits are used for data. bit 14 is set to select FREQ0 register
    writeADreg((1 << 14) | (freqWord & 0x3FFF)); //low word
    writeADreg((1 << 14) | ((freqWord >> 14) & 0x3FFF)); //high word
    writeADreg((1 << 15) | (1 << 14)); // Phase register 0
}

void writeADreg(uint16_t dat) {
    //SPI.setDataMode(SPI_MODE2);
    //TODO: add chip select support
    digitalWrite(FSYNC, LOW);
    SPI.transfer16(dat);
    digitalWrite(FSYNC, HIGH);
}