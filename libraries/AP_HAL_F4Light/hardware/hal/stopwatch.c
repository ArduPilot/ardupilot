#pragma GCC optimize ("O2")

#include <stm32f4xx.h>
#include <hal.h>
/* 
 * Provides a micro-second granular delay using the CPU cycle counter.
 */

/* cycles per microsecond */
uint32_t us_ticks;

void stopwatch_init(void)
{
	us_ticks = SystemCoreClock / 1000000;
	
	/* turn on access to the DWT registers */
	DEMCR |= DEMCR_TRCENA; 
	/* enable the CPU cycle counter */
	DWT_CTRL |= CYCCNTENA;	

        stopwatch_reset();
}


void stopwatch_delay_us(uint32_t us){
//	 we don't call stopwatch_reset() because any delay() in interrupt will reset main counter. It should be free running
    uint32_t ts = stopwatch_getticks(); // start time in ticks
    uint32_t dly = us * us_ticks;       // delay in ticks
    while(1) {
        uint32_t now = stopwatch_getticks(); // current time in ticks
        uint32_t dt = now - ts;

	if (dt >= dly)
		break;
    }
}
