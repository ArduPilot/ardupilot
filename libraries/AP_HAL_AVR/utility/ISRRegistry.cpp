#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <stdlib.h>
#include <avr/interrupt.h>

#include "ISRRegistry.h"

using namespace AP_HAL_AVR;

AP_HAL::Proc ISRRegistry::_registry[ISR_REGISTRY_NUM_SLOTS] = {NULL};

int ISRRegistry::register_signal(int signal, AP_HAL::Proc proc)
{
    if (signal >= 0 && signal < ISR_REGISTRY_NUM_SLOTS) {
        _registry[signal] = proc;
        return 0;
    }
    return -1;
}

int ISRRegistry::unregister_signal(int signal)
{
    if (signal >= 0 && signal < ISR_REGISTRY_NUM_SLOTS) {
        _registry[signal] = NULL;
        return 0;
    }
    return -1;
}

/* ========== ISR IMPLEMENTATIONS ========== */

extern "C" ISR(TIMER2_OVF_vect) {
    if ( ISRRegistry::_registry[ISR_REGISTRY_TIMER2_OVF] != NULL)
         ISRRegistry::_registry[ISR_REGISTRY_TIMER2_OVF]();
}


extern "C" ISR(TIMER4_CAPT_vect) {
    if ( ISRRegistry::_registry[ISR_REGISTRY_TIMER4_CAPT] != NULL)
         ISRRegistry::_registry[ISR_REGISTRY_TIMER4_CAPT]();
}

extern "C" ISR(TIMER5_CAPT_vect) {
    if ( ISRRegistry::_registry[ISR_REGISTRY_TIMER5_CAPT] != NULL)
         ISRRegistry::_registry[ISR_REGISTRY_TIMER5_CAPT]();
}

#endif
