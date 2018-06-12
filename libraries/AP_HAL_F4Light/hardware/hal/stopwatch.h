#ifndef _STOPWATCH_H
#define _STOPWATCH_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint32_t us_ticks;


#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))

#define DWT_CTRL	(*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA	(1<<0)
#define DWT_CYCCNT	((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

void stopwatch_init(void);
void stopwatch_delay_us(uint32_t us);

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


#ifdef __cplusplus
 }
#endif 
#endif
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
