
#ifndef __ARDUINO_MEGA_ISR_REGISTRY_H__
#define __ARDUINO_MEGA_ISR_REGISTRY_H__

#define ISR_REGISTRY_TIMER2_OVF    0
#define ISR_REGISTRY_TIMER4_CAPT   1
#define ISR_REGISTRY_TIMER5_CAPT   2
#define ISR_REGISTRY_NUM_SLOTS     3

typedef void (*proc_ptr)(void);

class Arduino_Mega_ISR_Registry
{
public:
    void                    init();
    int                     register_signal(int isr_number, proc_ptr proc);
    int                     unregister_signal(int isr_number);

    static proc_ptr         _registry[ISR_REGISTRY_NUM_SLOTS];
};

#endif // __ARDUINO_MEGA_ISR_REGISTRY_H__
