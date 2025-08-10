#ifndef _RADIO_H
#define _RADIO_H
#include <stdint.h>

void initRadio();
void sendRadio(float wt, float volts, int8_t state);
void WriteRegsCompact(void);
void sleepRadio();
#endif
