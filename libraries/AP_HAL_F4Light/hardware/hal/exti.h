#ifndef _EXTI_H_
#define _EXTI_H_

#include <string.h>
#include "hal_types.h"

#ifdef __cplusplus
  extern "C" {
#endif
 


#define EXTI_Line0       (1L<<0)     // External IRQ line 0 
#define EXTI_Line1       (1L<<1)     // External IRQ line 1 
#define EXTI_Line2       (1L<<2)     // External IRQ line 2 
#define EXTI_Line3       (1L<<3)     // External IRQ line 3 
#define EXTI_Line4       (1L<<4)     // External IRQ line 4 
#define EXTI_Line5       (1L<<5)     // External IRQ line 5 
#define EXTI_Line6       (1L<<6)     // External IRQ line 6 
#define EXTI_Line7       (1L<<7)     // External IRQ line 7 
#define EXTI_Line8       (1L<<8)     // External IRQ line 8 
#define EXTI_Line9       (1L<<9)     // External IRQ line 9 
#define EXTI_Line10      (1L<<10)    // External IRQ line 10 
#define EXTI_Line11      (1L<<11)    // External IRQ line 11 
#define EXTI_Line12      (1L<<12)    // External IRQ line 12 
#define EXTI_Line13      (1L<<13)    // External IRQ line 13 
#define EXTI_Line14      (1L<<14)    // External IRQ line 14 
#define EXTI_Line15      (1L<<15)    // External IRQ line 15 
#define EXTI_Line16      (1L<<16)    // External IRQ line 16 is PVD Output 
#define EXTI_Line17      (1L<<17)    // External IRQ line 17 is RTC Alarm event 
#define EXTI_Line18      (1L<<18)    // External IRQ line 18 is USB OTG FS Wakeup from suspend event 
#define EXTI_Line19      (1L<<19)    // External IRQ line 19 is Ethernet Wakeup event 
#define EXTI_Line20      (1L<<20)    // External IRQ line 20 is USB OTG HS (configured in FS) Wakeup event  
#define EXTI_Line21      (1L<<21)    // External IRQ line 21 is RTC Tamper and Time Stamp events 
#define EXTI_Line22      (1L<<22)    // External IRQ line 22 is RTC Wakeup event 


typedef enum
{
  EXTI_Trigger_Rising = 0x08,   // offset of EXTI_RTSR to use as address shift
  EXTI_Trigger_Falling = 0x0C,  // offset of EXTI_FTSR
  EXTI_Trigger_Rising_Falling = 0x10, // offset of EXTI_SWIER, but not used in address calculations
} EXTITrigger_t;

/**
 * External interrupt line numbers.
 */
typedef enum afio_exti_num {
    AFIO_EXTI_0,                // external IRQ line 0
    AFIO_EXTI_1,                // external IRQ line 1 
    AFIO_EXTI_2,                // external IRQ line 2 
    AFIO_EXTI_3,                // external IRQ line 3 
    AFIO_EXTI_4,                // external IRQ line 4 
    AFIO_EXTI_5,                // external IRQ line 5 
    AFIO_EXTI_6,                // external IRQ line 6 
    AFIO_EXTI_7,                // external IRQ line 7 
    AFIO_EXTI_8,                // external IRQ line 8 
    AFIO_EXTI_9,                // external IRQ line 9 
    AFIO_EXTI_10,               // external IRQ line 10 
    AFIO_EXTI_11,               // external IRQ line 11 
    AFIO_EXTI_12,               // external IRQ line 12 
    AFIO_EXTI_13,               // external IRQ line 13 
    AFIO_EXTI_14,               // external IRQ line 14 
    AFIO_EXTI_15,               // external IRQ line 15 
} afio_exti_num;

/**
 * external interrupt line port selector.
 *
 * Used to determine which GPIO port to map an external interrupt line
 * onto. */
/* (See AFIO sections, below) */
typedef enum afio_exti_port {
    AFIO_EXTI_PA,               // use port A
    AFIO_EXTI_PB,               // use port B
    AFIO_EXTI_PC,               // use port C
    AFIO_EXTI_PD,               // use port D
    AFIO_EXTI_PE,               // use port E
    AFIO_EXTI_PF,               // use port F
    AFIO_EXTI_PG,               // use port G
} afio_exti_port;

/** external interrupt trigger mode */
typedef enum exti_trigger_mode {
    EXTI_RISING         = EXTI_Trigger_Rising,         // trigger on the rising edge 
    EXTI_FALLING        = EXTI_Trigger_Falling,        // trigger on the falling edge 
    EXTI_RISING_FALLING = EXTI_Trigger_Rising_Falling, // trigger on both the rising and falling edges 
} exti_trigger_mode;

  
void exti_init();

/**
 * @brief Register a handler to run upon external interrupt.
 *
 * This function assumes that the interrupt request corresponding to
 * the given external interrupt is masked.
 *
 * @param num     External interrupt line number.
 * @param port    Port to use as source input for external interrupt.
 * @param handler Function handler to execute when interrupt is triggered.
 * @param mode    Type of transition to trigger on, one of:
 *                EXTI_RISING, EXTI_FALLING, EXTI_RISING_FALLING.
 * @see afio_exti_num
 * @see afio_exti_port
 * @see voidFuncPtr
 * @see exti_trigger_mode
 */
void exti_attach_interrupt_pri(afio_exti_num num,
                           afio_exti_port port,
                           Handler handler,
                           exti_trigger_mode mode,
                           uint8_t priority);


static inline void exti_attach_interrupt(afio_exti_num num,
                           afio_exti_port port,
                           Handler handler,
                           exti_trigger_mode mode)
{
    exti_attach_interrupt_pri(num, port, handler, mode, GPIO_INT_PRIORITY);
}


/**
 * @brief Unregister an external interrupt handler
 * @param num Number of the external interrupt line to disable.
 * @see afio_exti_num
 */
void exti_detach_interrupt(afio_exti_num num);

void exti_enable_irq(afio_exti_num num, bool e); // needed access to internal data

/**
 * Re-enable interrupts.
 *
 * Call this after noInterrupts() to re-enable interrupt handling,
 * after you have finished with a timing-critical section of code.
 *
 * @see noInterrupts()
 */
static INLINE void interrupts() {
        __enable_irq();
}
    
/**
 * Disable interrupts.
 *
 * After calling this function, all user-programmable interrupts will
 * be disabled.  You can call this function before a timing-critical
 * section of code, then call interrupts() to re-enable interrupt
 * handling.
 *
 * @see interrupts()
 */
static INLINE void noInterrupts() {
        __disable_irq();
}

static inline void exti_clear_pending_bit(uint32_t line)
{
  EXTI->PR = line;
}


void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
                       
#ifdef __cplusplus
  }
#endif
 

#endif
