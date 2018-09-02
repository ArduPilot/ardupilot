#pragma once

#include "hal_types.h"

#ifdef __cplusplus
extern "C"{
#endif

void enable_nvic_irq(uint8_t irq, uint8_t prio);


#ifdef __cplusplus
}
#endif