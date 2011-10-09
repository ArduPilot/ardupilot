#ifndef _AVR_INTERRUPT_H_
#define _AVR_INTERRUPT_H_

#include "io.h"

#define ISR(vector,...) extern "C" void vector(void);	\
void vector(void)

#define cli()
#define sei()

#endif
