#include "ad9833.h"
#include <Arduino.h>
#include <SPI.h>

using namespace AD9833;

void AD9833::init() {
    bitSet(DDRB, (FSYNC_PIN - 8)); //output
    SPI.begin();
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE2));
    delay(50);
    write(_BV(RESET));
    delay(50);
    write(_BV(13) | SQUARE); //B28 set so we can load FREQ0 in two writes

    setFreq(10000); //10khz
}

void AD9833::setFreq(uint32_t frequency) { //set AD9833 frequency
    //multiply by 40 because we divide by 40 for filter
    uint32_t freqWord = frequency * 40 * (268435456.0 / 25000000.0); //2^28 / crystal freq
    //only lower 14 bits are used for data
    write(_BV(FREQ0) | (freqWord & 0x3FFF)); //low word
    write(_BV(FREQ0) | ((freqWord >> 14) & 0x3FFF)); //high word
    write(_BV(15) | _BV(FREQ0)); // Phase register 0
}

void AD9833::write(uint16_t dat) {
    //SPI.setDataMode(SPI_MODE2);
    bitClear(PORTB, (FSYNC_PIN - 8)); //select
    SPI.transfer16(dat);
    bitSet(PORTB, (FSYNC_PIN - 8));
}