#pragma once

#include "hal_types.h"

#ifdef __cplusplus
extern "C"{
#endif

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
#define NVIC_Prio_group_0     (0x700) // 0 bits for pre-emption priority 4 bits for subpriority 
#define NVIC_Prio_group_1     (0x600) // 1 bits for pre-emption priority 3 bits for subpriority 
#define NVIC_Prio_group_2     (0x500) // 2 bits for pre-emption priority 2 bits for subpriority 
#define NVIC_Prio_group_3     (0x400) // 3 bits for pre-emption priority 1 bits for subpriority 
#define NVIC_Prio_group_4     (0x300) // 4 bits for pre-emption priority 0 bits for subpriority 

void enable_nvic_irq(uint8_t irq, uint8_t prio);


#ifdef __cplusplus
}
#endif