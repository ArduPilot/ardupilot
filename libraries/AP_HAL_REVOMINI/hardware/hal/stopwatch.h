#ifndef _STOPWATCH_H
#define _STOPWATCH_H

_BEGIN_STD_C

extern uint32_t us_ticks;

#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))

#define DWT_CTRL	(*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA	(1<<0)
#define DWT_CYCCNT	((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

void stopwatch_init(void);

static inline void stopwatch_reset(void)
{
   /* enable DWT access */
   DEMCR |= DEMCR_TRCENA; 
   *DWT_CYCCNT = 0;             
   /* enable the CPU cycle counter */
   DWT_CTRL |= CYCCNTENA;
}

static inline uint32_t stopwatch_getticks()
{
	return CPU_CYCLES;
}

static inline uint32_t stopwatch_getus()
{
	return CPU_CYCLES / us_ticks;
}

static inline uint32_t stopwatch_getms()
{
	return CPU_CYCLES / us_ticks / 1000;
}

static inline void stopwatch_delay_us(uint32_t us)
{
	stopwatch_reset();
	while(1)
	{
		if (stopwatch_getus() >= us)
			break;
	}
}

static inline void stopwatch_delay_ms(uint32_t ms)
{
	while (ms--) 
	{
 		stopwatch_delay_us(1000);
	}
}

_END_STD_C 
 
#endif
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
