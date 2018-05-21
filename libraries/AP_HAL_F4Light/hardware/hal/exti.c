/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: LeafLabs

*/

#pragma GCC optimize ("O2")

#include "exti.h"
#include "hal.h"
#include "util.h"
#include "nvic.h"
#include "rcc.h"


typedef struct exti_channel {
    uint32_t irq_line;
    IRQn_Type irq_type;
} exti_channel;


static const exti_channel exti_channels[] = {
    { .irq_line = EXTI_Line0,		.irq_type = EXTI0_IRQn      },  // EXTI0
    { .irq_line = EXTI_Line1,		.irq_type = EXTI1_IRQn      },  // EXTI1
    { .irq_line = EXTI_Line2,		.irq_type = EXTI2_IRQn      },  // EXTI2
    { .irq_line = EXTI_Line3,		.irq_type = EXTI3_IRQn      },  // EXTI3
    { .irq_line = EXTI_Line4,		.irq_type = EXTI4_IRQn      },  // EXTI4
    { .irq_line = EXTI_Line5,		.irq_type = EXTI9_5_IRQn    },  // EXTI5
    { .irq_line = EXTI_Line6,		.irq_type = EXTI9_5_IRQn    },  // EXTI6
    { .irq_line = EXTI_Line7,		.irq_type = EXTI9_5_IRQn    },  // EXTI7
    { .irq_line = EXTI_Line8,		.irq_type = EXTI9_5_IRQn    },  // EXTI8
    { .irq_line = EXTI_Line9,		.irq_type = EXTI9_5_IRQn    },  // EXTI9
    { .irq_line = EXTI_Line10,		.irq_type = EXTI15_10_IRQn  },  // EXTI10
    { .irq_line = EXTI_Line11,		.irq_type = EXTI15_10_IRQn  },  // EXTI11
    { .irq_line = EXTI_Line12,		.irq_type = EXTI15_10_IRQn  },  // EXTI12
    { .irq_line = EXTI_Line13,		.irq_type = EXTI15_10_IRQn  },  // EXTI13
    { .irq_line = EXTI_Line14,		.irq_type = EXTI15_10_IRQn  },  // EXTI14
    { .irq_line = EXTI_Line15,		.irq_type = EXTI15_10_IRQn  },  // EXTI15
};

#define NUM_IRQ (sizeof(exti_channels)/sizeof(exti_channel))

static Handler handlers[NUM_IRQ] IN_CCM;


void exti_init(){
    memset(handlers, 0, sizeof(handlers));
}


static inline EXTITrigger_t get_exti_mode(exti_trigger_mode mode) {
    switch (mode) {
    case EXTI_RISING:
        return EXTI_Trigger_Rising;
    case EXTI_FALLING:
        return EXTI_Trigger_Falling;
    case EXTI_RISING_FALLING:
        return EXTI_Trigger_Rising_Falling;
    }
    // Can't happen
    return (EXTITrigger_t)0;
}

void exti_attach_interrupt_pri(afio_exti_num num,
                           afio_exti_port port,
                           Handler handler,
                           exti_trigger_mode mode,
                           uint8_t priority)
{

    RCC_enableAPB2_clk(RCC_APB2_bit_SYSCFG); // we must wait some time before access to SYSCFG

    // register the handler 
    handlers[num] = handler;

    afio_exti_select(port, num); // select port as EXTI interrupt source. SYSCFG not documented in STM docs

    uint32_t line = exti_channels[num].irq_line;

    exti_clear_pending_bit(line); // clear active request if any
        
    EXTI->EMR &= ~line; // clear EXTI line in event register
    EXTI->IMR |= line;  // and set in IRQ register
    
    EXTITrigger_t trg = get_exti_mode(mode);    //  external interrupts trigger
    if (trg == EXTI_Trigger_Rising_Falling) {     // Rising & Falling edge 
        EXTI->RTSR |= line;
        EXTI->FTSR |= line;
    } else {
        // clears Rising/Falling edge
        EXTI->RTSR &= ~line; // Rising trigger selection register 
        EXTI->FTSR &= ~line; // Falling trigger selection register
        
        uint32_t addr = (uint32_t)EXTI_BASE + trg; // trg defined as offset to needed register
        *(__IO uint32_t *) addr |= line;
    }

    // Enable and set EXTI Line Interrupt to the given priority
    enable_nvic_irq(exti_channels[num].irq_type, priority);  // we init NVIC for 4 bit preemption,  0 bit subpriority 
}


void exti_detach_interrupt(afio_exti_num num)
{
    // disable external line
    uint32_t line = exti_channels[num].irq_line;

    EXTI->IMR &= ~line; // clear both registers
    EXTI->EMR &= ~line;

/* do not disable interrupt in NVIC because it can be shared  */

    handlers[num] = (Handler)0;	 // unregister the user's handler
}


void exti_enable_irq(afio_exti_num num, bool e){
    if(e){
         EXTI->IMR |= exti_channels[num].irq_line;
    }else {
         EXTI->IMR &= ~exti_channels[num].irq_line;

    }
}

/*
 * Interrupt handlers
 */


static void exti_serv(uint32_t extline, uint8_t num)
{
#ifdef ISR_PERF
    t = stopwatch_getticks();
#endif
    if((EXTI->IMR & extline)!=0 && (EXTI->PR & extline) !=0 ){
    
	Handler handler = handlers[num];

	if (handler) {
	    revo_call_handler(handler, num);
	}

	exti_clear_pending_bit(extline);
    }

#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif
}
    
void EXTI0_IRQHandler(void)
{
	exti_serv(EXTI_Line0, 0);
}

void EXTI1_IRQHandler(void) 
{
	exti_serv(EXTI_Line1, 1);
}

void EXTI2_IRQHandler(void) 
{
	exti_serv(EXTI_Line2, 2);
}

void EXTI3_IRQHandler(void) 
{
	exti_serv(EXTI_Line3, 3);
}

void EXTI4_IRQHandler(void) 
{
	exti_serv(EXTI_Line4, 4);
}

void EXTI9_5_IRQHandler(void) 
{
	exti_serv(EXTI_Line5, 5);
	exti_serv(EXTI_Line6, 6);
	exti_serv(EXTI_Line7, 7);
	exti_serv(EXTI_Line8, 8);
	exti_serv(EXTI_Line9, 9);
}

void EXTI15_10_IRQHandler(void)
{
	exti_serv(EXTI_Line10, 10);
	exti_serv(EXTI_Line11, 11);
	exti_serv(EXTI_Line12, 12);
	exti_serv(EXTI_Line13, 13);
	exti_serv(EXTI_Line14, 14);
	exti_serv(EXTI_Line15, 15);
}

