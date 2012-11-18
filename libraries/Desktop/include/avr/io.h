#ifndef _AVR_IO_H_
#define _AVR_IO_H_

#include <stdint.h>

#define F_CPU 16000000UL

#define _VECTOR(N) __vector_ ## N
#define _BV(bit) (1 << (bit))

extern "C" volatile uint8_t __iomem[1024];

#define _SFR_MEM8(addr)  __iomem[addr]
#define _SFR_MEM16(addr) (*(uint16_t *)&__iomem[addr])

#define _SFR_IO8(addr)  __iomem[addr]

extern "C" volatile uint8_t SREG;

#define _interrupts_are_blocked() ((SREG&0x80)==0)

#define __ATmegaxx0__
#include "iom2560.h"

#endif
