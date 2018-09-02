/*
    (c) 2017 night_ghost@ykoctpa.ru
 
*/

#include "nvic.h"

void enable_nvic_irq(uint8_t irq, uint8_t prio){
    NVIC_ClearPendingIRQ(irq);
    NVIC_EnableIRQ(irq);                // inline
    NVIC_SetPriority(irq, prio); // inline
}

