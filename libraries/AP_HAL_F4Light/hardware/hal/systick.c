/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: Leaflabs

*/
#pragma GCC optimize ("O2")


#include <systick.h>
#include <hal.h>
#include <timer.h>
#include "usart.h"
#include <string.h>

volatile uint64_t systick_uptime_millis IN_CCM;
voidFuncPtr boardEmergencyHandler IN_CCM;

#define MAX_SYSTICK_HANDLERS 4
static Handler systick_handlers[MAX_SYSTICK_HANDLERS] IN_CCM;
static uint8_t num_handlers=0;

#ifdef ISR_PROF
 uint64_t IN_CCM isr_time=0;
 uint32_t max_isr_time=0;
#endif

void systick_attach_callback(Handler callback) {  
    if(num_handlers<MAX_SYSTICK_HANDLERS) {
        systick_handlers[num_handlers++] = callback;
    }
}

void systick_detach_callback(Handler callback) {  
    uint8_t i;
    for(i=0; i< num_handlers; i++) {
        if(systick_handlers[i] == callback)
            systick_handlers[i] = 0;
    }
}

/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32_t reload_val) {
    memset(systick_handlers, 0, sizeof(systick_handlers));

    SysTick->LOAD = reload_val;
    systick_uptime_millis=0;
    systick_enable();
}


/*
 * SysTick ISR
 */

void SysTick_Handler(void)
{
    systick_uptime_millis++;

    uint8_t i;
    for(i=0; i< num_handlers; i++) {
        if(systick_handlers[i])
            revo_call_handler(systick_handlers[i], 0);
    }
}



// blinking on case of Faults

void __attribute__((noreturn)) error_throb(uint32_t num){
    int16_t  slope   = 1;
    uint16_t CC      = 0x0000;
    uint16_t TOP_CNT = 0x0200;
    uint16_t i       = 0;
    uint8_t n;

#ifdef BOARD_GPIO_B_LED_PIN
    const uint8_t pin= HAL_GPIO_B_LED_PIN;
#else
    const uint8_t pin= HAL_GPIO_A_LED_PIN;
#endif

    const stm32_pin_info *pp = &PIN_MAP[pin];

    
    /* Error fade. */
    while (1) {
        uint16_t k;
        for(k=0; k<num+1; k++) {
            uint32_t m;
            for(m=100000;m>0;m--){
                if (CC == TOP_CNT)  {
                    slope = -1;
                } else if (CC == 0) {
                    slope = 1;
                }
    
                if (i == TOP_CNT)  {
                    CC += slope;
                    i = 0;
                }

                if (i < CC) {
                    n=1;
                } else {
                    n=0;
                }
                gpio_write_bit(pp->gpio_device, pp->gpio_bit, n);

                volatile int j =10;
                while(--j);

                i++;
            }
        }
        emerg_delay(3000); // on 168MHz ~0.1ms so 300ms
    }
}




extern void __go_next_task();
extern void hal_try_kill_task_or_reboot(uint8_t num);
extern void hal_stop_multitask();
extern void hal_go_next_task();
#define NVIC_CCR (*(volatile uint32_t *)0xE000ED14)

// new common exception code
//
void __attribute__((noreturn)) __error(uint32_t num, uint32_t pc, uint32_t lr, uint32_t flag)
{

#ifdef DEBUG_BUILD
    static const char * const faults[] = {
        "", // 0 
        "", // 1
        "HardFault", // 2
        "MemManage fault", // 3 
        "BusFault", // 4 
        "UsageFault", // 5 
        "illegal Flash Write", // 6 
        "", // 7 
        "", // 8 
        "", // 9 
        "", // 10 
        "Pure Virtual function call", // 11
        "failed to setup clock", // 12
        "exit from main()", // 13
        "", // 14
    };
#endif


    /* Turn off peripheral interrupts */
    __disable_irq();


    extern bool hal_is_armed();
    if((flag & 0x4) && hal_is_armed()){ // not in interrupt and is armed - try to save aircraft
        hal_try_kill_task_or_reboot(num);
        // returned - task killed, resume normal operations
        NVIC_CCR &= ~1; // reset flag NONEBASETHRDENA in NVIC CCR
        __enable_irq();
        hal_go_next_task();
    }

    if(is_bare_metal())  // bare metal build without bootloader should reboot to DFU after any fault
        board_set_rtc_register(DFU_RTC_SIGNATURE, RTC_SIGNATURE_REG);

    timer_disable_all(); // turn off all PWM

    /* Turn the USB interrupt back on so "the reboot to bootloader" keeps on functioning */
    NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
    NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
    NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
    NVIC_EnableIRQ(OTG_HS_IRQn);
    NVIC_EnableIRQ(OTG_FS_IRQn);


    if(boardEmergencyHandler) boardEmergencyHandler(); // call emergency handler

    hal_stop_multitask();

#ifdef DEBUG_BUILD
    printf("\r\n!!! Exception: %s at %x LR=%x\n",faults[num], pc, lr);
#else
    printf("\r\n!!! Exception: %d at %x LR=%x\n",num, pc, lr);
#endif
    error_throb(num);
}

#if 0 // not removed for case if something will require Timer5

uint32_t systick_micros(void)
{
    volatile uint32_t fms, lms;
    uint32_t cycle_cnt;
     
    do {
        // make sure systick_uptime() return the same value before and after
        // getting the systick count
        fms = systick_uptime();
        cycle_cnt = systick_get_count();
        lms = systick_uptime();
    } while (lms != fms);

#define US_PER_MS               1000
    /* SYSTICK_RELOAD_VAL is 1 less than the number of cycles it
       actually takes to complete a SysTick reload */
    uint32_t res = (fms * US_PER_MS) +
        (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND;

    return res;
#undef US_PER_MS
}
#endif
