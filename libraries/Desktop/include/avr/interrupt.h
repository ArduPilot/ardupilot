#ifndef _AVR_INTERRUPT_H_
#define _AVR_INTERRUPT_H_

#include "io.h"

#define ISR(vector,...) void vector(void);	\
void vector(void)

extern "C" {
void cli(void);
void sei(void);
}

#endif
