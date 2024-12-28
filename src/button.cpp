#include <Arduino.h>
#include "button.h"

#define BOUNCE_TIME 50 //ignore taps shorter than this

Button::Button(uint8_t pin, void (*callback)(Button *button)){
    pinNum = pin;
    risingEdge = callback;
}

void Button::poll() {
    uint8_t buttonReading = ~PIND & _BV(pinNum);

    //restart debounce timer if button state changed
    if (buttonReading != lastState) tStamp = millis();
    lastState = buttonReading;

    if ((millis() - tStamp) > BOUNCE_TIME) {
        if(!debouncedState && buttonReading && risingEdge) //call rising edge function
            risingEdge(this);

        debouncedState = buttonReading;
    }
}