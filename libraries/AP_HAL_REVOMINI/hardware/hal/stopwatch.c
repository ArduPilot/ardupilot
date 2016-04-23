#include <stm32f4xx.h>
#include <hal.h>
/* 
 * Provides a micro-second granular delay using the CPU cycle counter.
 */

/* cycles per microsecond */
uint32_t us_ticks;

void stopwatch_init(void)
{
	RCC_ClocksTypeDef	clocks;
	
	/* compute the number of system clocks per microsecond */
	RCC_GetClocksFreq(&clocks);
	us_ticks = clocks.SYSCLK_Frequency / 1000000;
	
	//assert_param(us_ticks > 1);
	
	/* turn on access to the DWT registers */
	DEMCR |= DEMCR_TRCENA; 
	/* enable the CPU cycle counter */
	DWT_CTRL |= CYCCNTENA;	
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
