#pragma once
#include <stdint.h>

void initAD();
void writeADreg(uint16_t dat);
void setADfreq(uint32_t frequency);