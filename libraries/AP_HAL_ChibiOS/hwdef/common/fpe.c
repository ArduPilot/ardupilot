/*
  FPE detection support
 */

#include "hal.h"
#include "fpe.h"

uint32_t stm32_fpe_flags(void)
{
    // Flush to zero (IDC)
    // Inexact result (IXC)
    // Underflow (UFC)
    // Overflow (OFC)
    // Division by zero (DZC)
    // Invalid operation (IOC)

    // see page 14/31:
    const uint32_t IOC = 1U << 0;
    const uint32_t DZC = 1U << 1;
    const uint32_t OFC = 1U << 2;
    // const uint32_t UFC = 1U << 3;
    // const uint32_t IXC = 1U << 4;
    const uint32_t IDC = 1U << 7;

    return __get_FPSCR() & (IOC|DZC|OFC|IDC);
}
