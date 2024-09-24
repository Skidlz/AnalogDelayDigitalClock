#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include "ad9833.h"
#include "fastExpo.h"

#define DELAY_STEPS 4096
//Pin numbers
#define DLY_RATE_PN A0
#define LFO_RATE_PN A1
#define LFO_DEPT_PN A2

uint16_t nonblockAnalogRead(uint8_t pin);

uint8_t tick = 0; //timing flag

double ratePot = .3;
int16_t oldRatePot = 0; //use to detect pot changes
int16_t rawRatePot = 0;
uint16_t lfoRate = expoConvertInt(512);
float lfoDepth = .5;
int32_t lfoPhase = 0; //phase accumulator for NCO
bool lfoDir = false;
#define LFOMAX ((long)1 << 18) -1 //bottom 8 bits discarded

void setup() {
    DDRD = 0; //all input
    PORTD = 0xff; //pullup resistors
    DDRB = 0b00111111;

    initAD(); //setup AD9833

    //setup timer to time LFO steps and AD9833 updates---------------------------------------------
    noInterrupts();
    TCCR1A = TCCR1B = TCNT1 = 0;
    TCCR1B = (1 << CS11)|(1 << CS10)|(1 << WGM12); //div 64 16MHz/64=250kHz
    OCR1A = (250000/3500) - 1; //3.5kHz
    bitSet(TIMSK1, OCIE1A);
    interrupts();
}

ISR(TIMER1_COMPA_vect) { tick = 1; }

//tap tempo variables------------------------------------------------------------------------------
uint32_t prevTS = 0;
uint32_t lastTS = 0; //last toggled time stamp
uint8_t lastBtnState = 0; //the previous reading from the input pin
uint8_t btnState = 0;
uint32_t tapFreq = 0;
#define debounceDelay 50 //ignore taps shorter than this
#define tapTimeout 2500 //ignore taps that take longer than this

uint8_t useTapRate = 0;
float tapExpo = 0;
float tapEquivPotValue = 0;
#define rateMultiplier 90000 //sets upper limit of delay rate
#define rateOffset 3000 //sets lower limit of delay rate

uint8_t pollTap() {
    uint8_t buttonReading = ~PIND & (1 << 7);

    //restart debounce timer if button state changed
    if (buttonReading != lastBtnState) lastTS = millis();
    lastBtnState = buttonReading;

    if ((millis() - lastTS) > debounceDelay) {
        if(!btnState && buttonReading) { //rising edge
            uint32_t tapPeriod = lastTS - prevTS;
            if (tapPeriod < tapTimeout) { //make sure we're under the tapTimeout limit
                //convert from tap time to frequency based on delay line length
                tapFreq = (uint32_t) DELAY_STEPS * 1000 / tapPeriod;
                setADfreq(tapFreq);
                float tapLinear = ((float) (tapFreq  - rateOffset) / rateMultiplier); //normalize to 1 (roughly)
                //convert tap time back into pot position
                //inverse of y=0.0125 * 81^x - 0.0125
                tapEquivPotValue = log(1 + 80 * tapLinear) / log(81);

                //store calculated expo rate so we can subtract it from the LFO later
                tapExpo = expoConvert(tapEquivPotValue); //3k - 93k

                useTapRate = 1; //switch over to tap tempo
                oldRatePot = rawRatePot; //if pot deviates too much, then go back to using pot value
            }

            prevTS = lastTS;
        }

        btnState = buttonReading;
    }
}


void updateRatePot() {
    rawRatePot = nonblockAnalogRead(DLY_RATE_PN); //Delay rate
    ratePot = rawRatePot / 1024.0; //standardize to 0-1.0

    //if rate changes enough, use ratePot value not tapRate
    if (abs(rawRatePot - oldRatePot) > 4) useTapRate = 0;
}

void loop() {
    pollTap();

    updateRatePot();

    lfoRate = expoConvertInt(nonblockAnalogRead(LFO_RATE_PN));

    updateRatePot(); //update delay more frequently for smoother sweeps

    lfoDepth = expoConvert(nonblockAnalogRead(LFO_DEPT_PN) / 1024.0); //standardize to expo 0-1.0
}

uint16_t nonblockAnalogRead(uint8_t pin) { //taken from analogRead
    uint8_t low, high;

    if (pin >= 14) pin -= 14; // allow for channel or pin numbers

    // set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits).  this also sets ADLAR (left-adjust result) to 0 (the default).
    uint8_t analog_reference = 1; //default
    ADMUX = (analog_reference << 6) | (pin & 0x07);

    bitSet(ADCSRA, ADSC); // start the conversion

    while (bit_is_set(ADCSRA, ADSC)) { // ADSC is cleared when the conversion finishes
        if (tick) { //wait for interrupt
            tick = 0;

            if (useTapRate) {
                //expo scale LFO with linear rate offset
                double expoLFO = expoConvert(tapEquivPotValue + (lfoPhase / 512) * lfoDepth / 4096);
                expoLFO -= tapExpo; //subtract *expo* offset
                expoLFO = expoLFO * rateMultiplier;
                setADfreq(tapFreq + expoLFO); //3k - 93k
            } else {
                double expoPot = expoConvert(ratePot + (lfoPhase / 512) * lfoDepth / 4096);
                setADfreq(rateOffset + expoPot * rateMultiplier); //3k - 93k
            }

            //step LFO
            if (lfoDir) {
                lfoPhase -= lfoRate;
                if (lfoPhase <= -(long int) LFOMAX) {
                    lfoPhase = -LFOMAX - (lfoPhase + LFOMAX);
                    lfoDir = !lfoDir; //change direction
                }
            } else {
                lfoPhase += lfoRate;
                if (lfoPhase >= LFOMAX) {
                    lfoPhase = LFOMAX - (lfoPhase - LFOMAX);
                    lfoDir = !lfoDir; //change direction
                }
            }
        }
    };

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    low  = ADCL;
    high = ADCH;

    // combine the two bytes
    return (high << 8) | low;
}
