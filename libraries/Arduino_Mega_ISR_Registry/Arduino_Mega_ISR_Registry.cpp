
#include <stdlib.h>
#include "Arduino_Mega_ISR_Registry.h"
#include "avr/interrupt.h"

proc_ptr Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_NUM_SLOTS];

void Arduino_Mega_ISR_Registry::init()
{
    for (int i = 0; i < ISR_REGISTRY_NUM_SLOTS; i++)
        _registry[i] = NULL;
}

int Arduino_Mega_ISR_Registry::register_signal(int signal, proc_ptr proc)
{
    if (signal >= 0 && signal < ISR_REGISTRY_NUM_SLOTS) {
        _registry[signal] = proc;
        return 0;
    }
    return -1;
}

int Arduino_Mega_ISR_Registry::unregister_signal(int signal)
{
    if (signal >= 0 && signal < ISR_REGISTRY_NUM_SLOTS) {
        _registry[signal] = NULL;
        return 0;
    }
    return -1;
}

/* ========== ISR IMPLEMENTATIONS ========== */

extern "C" ISR(TIMER2_OVF_vect) {
    if (Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_TIMER2_OVF] != NULL)
        Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_TIMER2_OVF]();
}


extern "C" ISR(TIMER4_CAPT_vect) {
    if (Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_TIMER4_CAPT] != NULL)
        Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_TIMER4_CAPT]();
}

extern "C" ISR(TIMER5_CAPT_vect) {
    if (Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_TIMER5_CAPT] != NULL)
        Arduino_Mega_ISR_Registry::_registry[ISR_REGISTRY_TIMER5_CAPT]();
}
