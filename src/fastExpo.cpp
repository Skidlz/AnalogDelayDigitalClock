#include "fastExpo.h"

//Expo conversion functions based on https://electronics.stackexchange.com/a/341052 ---------------
typedef struct { float slope; float offset; } expoPiecewise;
expoPiecewise expoLookup[8] = { //array of lines approximating y=0.0125 * 81^x - 0.0125
        {0.07321, 0.00000},
        {0.12679, -0.00670},
        {0.21962, -0.02990},
        {0.38038, -0.09019},
        {0.65885, -0.22942},
        {1.14115, -0.53087},
        {1.97654, -1.15740},
        {3.42346, -2.42346}
};

float expoConvert(float linVal) {
    uint8_t index = linVal * 8;
    if (index >= 8) index = 7;

    return linVal * expoLookup[index].slope + expoLookup[index].offset;
}

typedef struct { int16_t slope; int16_t offset; } expoPiecewiseInt;
expoPiecewiseInt expoLookupInt[8] = { //array of lines approximating y=0.0125 * 81^x - 0.0125
        {600, 0},
        {1039, -55},
        {1799, -245},
        {3116, -739},
        {5397, -1879},
        {9348, -4349},
        {16192, -9481},
        {28045, -19853},
};

uint16_t expoConvertInt(uint32_t linVal) { // 0 - 1024 value
    uint8_t index = linVal / 128; //truncate to just 8 values
    if (index >= 8) index = 7;

    return (linVal * expoLookupInt[index].slope) / 1024 + expoLookupInt[index].offset + 1;
}