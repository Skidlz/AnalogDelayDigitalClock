#include <Arduino.h>
#include "ad9833.h"
#include "fastExpo.h"
#include "midi.h"
#include "button.h"

#define DELAY_STEPS 4096 //V3205
#define RATE_MULT 90000 //sets upper limit of delay rate
#define RATE_OFST 3000 //sets lower limit of delay rate

//Pin numbers
#define DLY_RATE_PN A0
#define LFO_RATE_PN A1
#define LFO_DEPT_PN A2
#define TAP_LED_PN 2
#define LFO_LED_PN 3
#define LFO_PHS_PN 4
#define ON_LED_PN 5
#define BYPASS_PN 6
#define TAP_PN 7

//functions--------------------------------------
uint16_t nonblockAnalogRead(uint8_t pin);
uint32_t setTempoToMS(uint32_t tapPeriod);
void midiClockHandler();
void midiNoteHandler(uint8_t note, uint8_t vel);
void midiCcHandler(uint8_t cc, uint8_t val);
void tapHandler(Button *button);
void updateLED();
void updateRatePot();
void stepLFO();

enum ClockSources { POT, TAP, CLOCK, MIDI_CLOCK, MIDI_CC };
ClockSources clockSource = POT;

MIDI midi;
Button tapButton(TAP_PN, tapHandler);

//pot values-------------------------------------
double ratePot = .3;
uint16_t lfoRate = expoConvertInt(512);
float lfoDepth = .5;
int16_t oldRatePot = 0; //use to detect pot changes
int16_t rawRatePot = 0;

uint32_t tapRaw; //used for noteMultiplier

//lfo variables----------------------------------
uint8_t tick = 0; //timing flag
int32_t lfoPhase = 0; //phase accumulator for NCO
#define LFOMAX ((long)1 << 18) -1

//led variables----------------------------------
uint32_t nextLedTS = 0; //scheduled LED toggle time
uint32_t ledInterval = 0;

ISR(TIMER1_COMPA_vect) { tick = 1; }

void setup() {
    DDRD = _BV(TAP_LED_PN) | _BV(LFO_LED_PN) | _BV(LFO_PHS_PN) | _BV(ON_LED_PN); //LED outputs
    PORTD = _BV(TAP_PN); //pullup resistor

    Serial.begin(31250); //MIDI baud
    midi.rtClock = midiClockHandler; //callbacks
    midi.noteOn = midiNoteHandler;
    midi.controlChange = midiCcHandler;

    AD9833::init(); //setup AD9833

    //setup timer to time LFO steps and AD9833 updates---------------------------------------------
    noInterrupts();
    TCCR1A = TCCR1B = TCNT1 = 0;
    TCCR1B = _BV(CS11)|_BV(CS10)|_BV(WGM12); //div 64 16MHz/64=250kHz
    OCR1A = (250000/4000) - 1; //4kHz LFO tick
    bitSet(TIMSK1, OCIE1A);
    interrupts();

    analogWrite(LFO_LED_PN, 128); //init LFO led
}

void loop() { //loops at about 1kHz
    tapButton.poll();

    updateLED();

    updateRatePot();

    lfoRate = expoConvertInt(nonblockAnalogRead(LFO_RATE_PN));

    updateRatePot(); //update delay more frequently for smoother sweeps

    lfoDepth = expoConvert(nonblockAnalogRead(LFO_DEPT_PN) / 1024.0); //standardize to expo 0-1.0
}

float noteTranspose = 1.0; //MIDI note transpose multiplier
#define MID_C 60 //MIDI note for middle C
#define NOTE_LEN 49 //length of note table
const float noteOffsets[NOTE_LEN] PROGMEM = { //2^(n/12) -24 <= n <= 24
        0.2500, 0.2649, 0.2806, 0.2973, 0.3150, 0.3337, 0.3536, 0.3746, 0.3969, 0.4204, 0.4454, 0.4719, //-2 oct
        0.5000, 0.5297, 0.5612, 0.5946, 0.6300, 0.6674, 0.7071, 0.7492, 0.7937, 0.8409, 0.8909, 0.9439, //-1 oct
        1.0000, 1.0595, 1.1225, 1.1892, 1.2599, 1.3348, 1.4142, 1.4983, 1.5874, 1.6818, 1.7818, 1.8877, //middle
        2.0000, 2.1189, 2.2449, 2.3784, 2.5198, 2.6697, 2.8284, 2.9966, 3.1748, 3.3636, 3.5636, 3.7755, //+1 oct
        4.0000 }; //+2 oct

void midiNoteHandler(uint8_t note, uint8_t vel) {
    int8_t offsetIndex = note - MID_C + 24; //middle c is at middle of note table
    if (offsetIndex < 0) offsetIndex = 0;
    else if (offsetIndex > NOTE_LEN - 1) offsetIndex = NOTE_LEN - 1;

    noteTranspose = pgm_read_float_near(noteOffsets + offsetIndex);
    if (clockSource == TAP || clockSource == MIDI_CLOCK)
        setTempoToMS(tapRaw);
}

//TODO: quantize clocks to combat jitter
void midiClockHandler() {
    static uint32_t prevMidiTs;
    static uint8_t clockCount = 0;

    //measure time between 12 clocks and convert them into a tempo (MIDI is 24ppqn)
    uint32_t lastTS = millis();
    if (clockCount++ == 0)
        prevMidiTs = lastTS;
    else if (clockCount == 12) {
        uint32_t newTapRaw = setTempoToMS((lastTS - prevMidiTs) * 2);
        if (newTapRaw) tapRaw = newTapRaw; //store for noteTranspose
        clockCount = 0;
    }

    //TODO: add option to ignore incoming clock
    clockSource = MIDI_CLOCK; //switch over to MIDI clock
}

void midiCcHandler(uint8_t cc, uint8_t val) {
    //TODO: override pot values with cc value
    switch(cc) {
        case 16: //delay time
            break;
        case 17: //LFO rate
            break;
        case 18: //lfo amount
            break;
    }
}

//tap tempo variables------------------------------------------------------------------------------
uint32_t tapFreq = 0;
float tapExpo = 0;
float tapEquivPotValue = 0;
#define TAP_TIMEOUT 2500 //ignore taps that take longer than this

//TODO: add support for different divisions 8ths, dotted 8ths, triplet 8ths, 16ths
uint32_t setTempoToMS(uint32_t tapPeriod) {
    //limit slow or very fast clocks > 200kHz
    if (tapPeriod > TAP_TIMEOUT || tapPeriod < 20) return 0; //don't pass validation

    //convert from tap time to frequency based on delay line length
    tapFreq = (uint32_t) DELAY_STEPS * 1000 / (tapPeriod / noteTranspose);

    float tapLinear = ((float) (tapFreq - RATE_OFST) / RATE_MULT); //normalize to 1 (roughly)
    //convert tap time back into pot position
    //inverse of y=0.0125 * 81^x - 0.0125
    tapEquivPotValue = log(1 + 80 * tapLinear) / log(81);
    //store calculated expo rate so we can subtract it from the LFO later
    tapExpo = expoConvert(tapEquivPotValue); //3k - 93k

    oldRatePot = rawRatePot; //if pot deviates too much, then go back to using pot value

    return tapPeriod; //passed validation
}

void updateLED() {
    if (millis() >= nextLedTS) { //tap LED
        PORTD ^= _BV(TAP_LED_PN); //toggle
        nextLedTS += ledInterval; //setup next TS
    }

    //LFO lED------------------------------------
    OCR2B = lfoPhase >> 10; //top 8 bits set intensity, sign sets color
    (lfoPhase < 0) ? bitSet(PORTD, LFO_PHS_PN) : bitClear(PORTD, LFO_PHS_PN);
}

void tapHandler(Button *button) { //called on debounced rising edge
    static uint32_t prevTS = 0;

    noteTranspose = 1.0; //reset MIDI note offset
    uint32_t newTapRaw = setTempoToMS(button->tStamp - prevTS);
    if (newTapRaw) tapRaw = newTapRaw; //store for noteMultiplier

    clockSource = TAP; //switch over to tap tempo

    prevTS = button->tStamp;
}

void updateRatePot() {
    rawRatePot = nonblockAnalogRead(DLY_RATE_PN); //Delay rate
    ratePot = rawRatePot / 1024.0; //standardize to 0-1.0

    //if rate changes enough, use ratePot value not tapRate
    if (abs(rawRatePot - oldRatePot) > 4) {
        clockSource = POT;
        noteTranspose = 1.0; //reset MIDI note offset
        oldRatePot = rawRatePot;
    }
}

void stepLFO() {
    static bool lfoDir = false;

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

uint16_t nonblockAnalogRead(uint8_t pin) { //taken from analogRead
    if (pin >= 14) pin -= 14; //allow for channel or pin numbers

    //set the analog reference (high two bits of ADMUX) and select the
    //channel (low 4 bits).  this also sets ADLAR (left-adjust result) to 0 (the default).
    uint8_t analog_reference = 1; //default
    ADMUX = (analog_reference << 6) | (pin & 0x07);

    bitSet(ADCSRA, ADSC); //start the conversion

    while (bit_is_set(ADCSRA, ADSC)) { //ADSC is cleared when the conversion finishes
        if (Serial.available()) midi.handleByte(Serial.read());

        if (tick) { //wait for LFO interrupt
            tick = 0;
            uint32_t newFreq = 0;

            //update AD9833--------------------------------
            if (clockSource == TAP || clockSource == MIDI_CLOCK) {
                //expo scale LFO with linear rate offset
                double expoLFO = expoConvert(tapEquivPotValue + (lfoPhase * lfoDepth) / ((long)1 << 21));
                expoLFO -= tapExpo; //subtract *expo* offset
                expoLFO = expoLFO * RATE_MULT;
                newFreq = tapFreq + expoLFO;
            } else { //Pot
                double expoPot = expoConvert(ratePot + (lfoPhase * lfoDepth) / ((long)1 << 21));
                newFreq = (RATE_OFST + expoPot * RATE_MULT) * noteTranspose;
            }

            AD9833::setFreq(newFreq); //3k - 93k

            //calc new LED interval------------------------
            uint32_t newLedInterval = ((uint32_t)DELAY_STEPS * (1000 / 4)) / newFreq;

            if (nextLedTS >= ledInterval) //prevent underflow
                nextLedTS = nextLedTS - ledInterval + newLedInterval; //replace old interval with new

            ledInterval = newLedInterval;

            stepLFO();
        }
    };

    uint8_t low  = ADCL; //have to read ADCL first to "lock" ADCH result
    uint8_t high = ADCH;
    return (high << 8) | low; // combine the two bytes
}