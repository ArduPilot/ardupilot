
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <util/delay.h>
#include <avr/interrupt.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#define AVR_TIMER_TCNT TCNT5;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1      
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#define AVR_TIMER_TCNT TCNT4;
#endif

inline uint16_t local_ticks( uint16_t &ticksTimer ) {
  uint8_t _sreg = SREG;
  cli();
  uint16_t tcnt = AVR_TIMER_TCNT;
  SREG = _sreg;
  uint16_t old = ticksTimer;
  ticksTimer = tcnt;
  if( tcnt < old ) tcnt += 40000; // TCNT wrap
  return tcnt - old;
}

inline uint16_t local_ticks( uint16_t ticksTimer, uint16_t ticksDelay ) {
  uint8_t _sreg = SREG;
  cli();
  uint16_t tcnt = AVR_TIMER_TCNT;
  SREG = _sreg;
  uint16_t old = ticksTimer;
  if( tcnt < old ) tcnt += 40000; // TCNT wrap
  tcnt -= old;  
  if( tcnt < ticksDelay ) return ticksDelay - tcnt;
  return 0;
}

#define repCount 1000

volatile uint16_t foo_int;
volatile float foo_float;
volatile uint32_t bar; 

void loop( void ) {
}

void setup(void) {
    float whileCost;
    float result;

    uint16_t timer;
    uint16_t ticks;

    hal.console->printf_P( PSTR("\nticksDemo:\n") );
    hal.console->printf_P( PSTR("---------------------------------\n") );

    uint16_t rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
      __asm( "nop" ); // Must be here to prevent compiler from otimizing away while loop code.
    }
    ticks = hal.scheduler->ticks( timer );
    ticks -= repCount / (F_CPU / 1000 / 1000 / 2); // Remove NOP command time
    whileCost = ticks / (repCount * 2.0f);
    whileCost *= 1000.0f;
    hal.console->printf_P( PSTR("while() loop iteration time = %fns\n"), whileCost );
    hal.console->printf_P( PSTR("This time will be subtracted from results.\n") );
   
    // Reference 1us
    // ----------------------------------------------------------------------------------------------
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        _delay_us( 1 );
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("\nTest reference 1000ns = %fns\n"), result );

    hal.console->printf_P( PSTR("\nMath average execution time:\n") );
    hal.console->printf_P( PSTR("---------------------------------\n") );

    // Math average execution time
    // ----------------------------------------------------------------------------------------------
    // Integer fixed-point multiplication
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        foo_int = rep * 123456;
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("Integer fixed point multiplication = %fns\n"), result );

    // Integer decimal point multiplication
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        foo_int = rep * 123.456f;
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("Integer decimal point multiplication = %fns\n"), result );

    // Float multiplication
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        foo_float = (float)rep * 123456.0f;
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("Float multiplication = %fns\n"), result );


    // Integer fixed point division
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        foo_int = (uint16_t)rep / 123456;
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("Integer fixed point division = %fns\n"), result );

    // Integer decimal point division
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        foo_int = (uint16_t)rep / 123.456f;
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("Integer decimal point division = %fns\n"), result );

    // Float division
    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        foo_float = (float)rep / 123456.0f;
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    result *= 1000.0f;
    result -= whileCost;
    hal.console->printf_P( PSTR("Float division = %fns\n"), result );

    // Code profiling
    // ----------------------------------------------------------------------------------------------
    uint16_t timerTotal;
    hal.scheduler->ticks( timerTotal );

      uint16_t timerA;
      hal.scheduler->ticks( timerA );
        // Code to profile
        _delay_us( 100 );
      uint16_t resultA = hal.scheduler->ticks( timerA );
      
      uint16_t timerB;
      hal.scheduler->ticks( timerB );
        // Code to profile
        _delay_us( 200 );
      uint16_t resultB = hal.scheduler->ticks( timerB );

    uint16_t resultTotal = hal.scheduler->ticks( timerTotal );
    
    hal.console->printf_P( PSTR("\nCode profile:\n") );
    hal.console->printf_P( PSTR("---------------------------------\n") );
    hal.console->printf_P( PSTR("Code A = %fus\n"), resultA / 2.0f );
    hal.console->printf_P( PSTR("Code B = %fus\n"), resultB / 2.0f );    
    hal.console->printf_P( PSTR("Code total = %fus\n"), resultTotal / 2.0f );

    local_ticks( timerTotal );

      local_ticks( timerA );
        // Code to profile
        _delay_us( 100 );
      resultA = local_ticks( timerA );
      
      local_ticks( timerB );
        // Code to profile
        _delay_us( 200 );
      resultB = local_ticks( timerB );

    resultTotal = local_ticks( timerTotal );
    
    hal.console->printf_P( PSTR("\nCode profile (local)):\n") );
    hal.console->printf_P( PSTR("---------------------------------\n") );
    hal.console->printf_P( PSTR("Code A = %fus\n"), resultA / 2.0f );
    hal.console->printf_P( PSTR("Code B = %fus\n"), resultB / 2.0f );    
    hal.console->printf_P( PSTR("Code total = %fus\n"), resultTotal / 2.0f );


    // Code timing
    // ----------------------------------------------------------------------------------------------
    hal.console->printf_P( PSTR("\nCode timing:\n") );
    hal.console->printf_P( PSTR("---------------------------------\n") );

    uint16_t timerWait;
    uint16_t resultWait;
    uint32_t waitTrigger;
    
    hal.scheduler->ticks( timerWait );
      // millis() timing
      waitTrigger = hal.scheduler->millis() + 10.25 * 1;
      while( waitTrigger > hal.scheduler->millis() );
    resultWait = hal.scheduler->ticks( timerWait );
    hal.console->printf_P( PSTR("millis() wait 10.25ms = %fms\n"), resultWait / 2000.0f );

    hal.scheduler->ticks( timerWait );
      // micros() timing
      waitTrigger = hal.scheduler->micros() + 10.25 * 1000;
      while( waitTrigger > hal.scheduler->micros() );
    resultWait = hal.scheduler->ticks( timerWait );
    hal.console->printf_P( PSTR("micros() wait 10.25ms = %fms\n"), resultWait / 2000.0f );

    hal.scheduler->ticks( timerWait );
      // ticks timing()
      while( hal.scheduler->ticks( timerWait, 10.25 * 2000 ) );
    resultWait = hal.scheduler->ticks( timerWait );
    hal.console->printf_P( PSTR("ticks() wait 10.25ms = %fms\n"), resultWait / 2000.0f );

    local_ticks( timerWait );
      // ticks timing()
      while( local_ticks( timerWait, 10.25 * 2000 ) );
    resultWait = local_ticks( timerWait );
    hal.console->printf_P( PSTR("Local ticks() wait 10.25ms = %fms\n"), resultWait / 2000.0f );


    // Timer functions execution time
    // ----------------------------------------------------------------------------------------------
    hal.console->printf_P( PSTR("\nTimer functions execution time:\n") );
    hal.console->printf_P( PSTR("---------------------------------\n") );

    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        bar = hal.scheduler->millis();
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    hal.console->printf_P( PSTR("millis() = %fus\n"), result );

    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        bar = hal.scheduler->micros();
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    hal.console->printf_P( PSTR("micros() = %fus\n"), result );

    rep = repCount;
    hal.scheduler->ticks( timer );      
    while( --rep )
    {
        uint16_t tmp;
        bar = hal.scheduler->ticks( tmp );
    }
    ticks = hal.scheduler->ticks( timer );
    result = ticks / (repCount * 2.0f);
    hal.console->printf_P( PSTR("ticks() = %fus\n"), result );

    rep = repCount;
    local_ticks( timer );      
    while( --rep )
    {
        uint16_t tmp;
        bar = local_ticks( tmp );
    }
    ticks = local_ticks( timer );
    result = ticks / (repCount * 2.0f);
    hal.console->printf_P( PSTR("Local ticks() = %fus\n"), result );
}

AP_HAL_MAIN();

