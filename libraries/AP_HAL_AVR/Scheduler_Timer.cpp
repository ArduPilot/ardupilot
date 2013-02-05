#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Scheduler.h"
using namespace AP_HAL_AVR;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

static volatile uint32_t timer_micros_counter = 0;
static volatile uint32_t timer_millis_counter = 0;

void AVRTimer::init() {
    // this needs to be called before setup() or some functions won't
    // work there
    sei();

    // set a2d prescale factor to 128
    // 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
    // XXX: this will not work properly for other clock speeds, and
    // this code should use F_CPU to determine the prescale factor.
    sbi(ADCSRA, ADPS2);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS0);

    // enable a2d conversions
    sbi(ADCSRA, ADEN);

    // the bootloader connects pins 0 and 1 to the USART; disconnect them
    // here so they can be used as normal digital i/o; they will be
    // reconnected in Serial.begin()
    UCSR0B = 0;
}

#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 )
#define  AVR_TIMER_OVF_VECT     TIMER4_OVF_vect
#define  AVR_TIMER_TCNT             TCNT4
#define  AVR_TIMER_TIFR              TIFR4
#elif (CONFIG_HAL_BOARD == HAL_BOARD_APM2 )
#define  AVR_TIMER_OVF_VECT     TIMER5_OVF_vect 
#define  AVR_TIMER_TCNT             TCNT5
#define  AVR_TIMER_TIFR              TIFR5
#endif

SIGNAL( AVR_TIMER_OVF_VECT)
{
    // Hardcoded for AVR@16MHZ and 8x pre-scale 16-bit timer overflow at 40000
    timer_micros_counter += 40000 / 2; // 20000us each overflow
    timer_millis_counter += 40000 / 2000; // 20ms each overlflow
}

uint32_t AVRTimer::micros() {
    uint8_t oldSREG = SREG;
	cli();

    // Hardcoded for AVR@16MHZ and 8x pre-scale 16-bit timer
    //uint32_t time_micros = timer_micros_counter + (AVR_TIMER_TCNT / 2);
    //uint32_t time_micros = timer_micros_counter + (AVR_TIMER_TCNT >> 1);
    
    uint32_t time_micros = timer_micros_counter;
    uint16_t tcnt = AVR_TIMER_TCNT;
	
    // Check for  imminent timer overflow interrupt and pre-increment counter
    if ( AVR_TIMER_TIFR & 1 && tcnt < 39999 )
    {
            time_micros += 40000 / 2;
    }
    SREG = oldSREG;

	return  time_micros + (tcnt >> 1);
}

uint32_t AVRTimer::millis() {
	uint8_t oldSREG = SREG;
	cli();
    // Hardcoded for AVR@16MHZ and 8x pre-scale 16-bit timer
    //uint32_t time_millis = timer_millis_counter + (AVR_TIMER_TCNT / 2000) ;
    //uint32_t time_millis =  timer_millis_counter + (AVR_TIMER_TCNT >> 11); // AVR_TIMER_CNT / 2048 is close enough (24us counter delay)

    uint32_t time_millis =  timer_millis_counter;
    uint16_t tcnt = AVR_TIMER_TCNT;
    
    // Check for imminent timer overflow interrupt and pre-increment counter
    if ( AVR_TIMER_TIFR & 1 && tcnt < 39999 )
    {
            time_millis += 40000 / 2000;
    }
    SREG = oldSREG;

	return  time_millis + (tcnt >> 11);
}


/* Delay for the given number of microseconds.  Assumes a 16 MHz clock. */
void AVRTimer::delay_microseconds(uint16_t us)
{
	// for the 16 MHz clock on most Arduino boards
	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}

#endif
