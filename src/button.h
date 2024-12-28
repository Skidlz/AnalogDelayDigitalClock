#include <stdio.h>
#pragma once

class Button {
public:
    uint8_t pinNum; //pin # on PIND
    uint8_t debouncedState;
    uint32_t tStamp; //last toggled time stamp
    void (*risingEdge)(Button *button);

    Button(uint8_t pin, void (*callback)(Button *button));
    void poll();
private:
    uint8_t lastState;  //previous reading from the input pin
};