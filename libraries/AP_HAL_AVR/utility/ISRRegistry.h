

#ifndef __AP_HAL_AVR_ISR_REGISTRY_H__
#define __AP_HAL_AVR_ISR_REGISTRY_H__

#include "AP_HAL_AVR_Namespace.h"

#define ISR_REGISTRY_TIMER2_OVF  0
#define ISR_REGISTRY_TIMER4_CAPT 1
#define ISR_REGISTRY_TIMER5_CAPT 2
#define ISR_REGISTRY_NUM_SLOTS   3

typedef void (*proc_ptr)(void);

class AP_HAL_AVR::ISRRegistry {
public:
    int register_signal(int isr_number, proc_ptr proc);
    int unregister_signal(int isr_number);
    
    static proc_ptr _registry[ISR_REGISTRY_NUM_SLOTS];
};

#endif // __AP_HAL_AVR_ISR_REGISTRY_H__
