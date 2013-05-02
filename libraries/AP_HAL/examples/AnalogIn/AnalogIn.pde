
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::AnalogSource* ch;

void setup (void) {
    hal.console->printf_P(PSTR("Starting AP_HAL::AnalogIn test\r\n"));
    ch = hal.analogin->channel(0);
}

static int8_t pin;

void loop (void) 
{
    float v  = ch->voltage_average(); 
    if (pin == 0) {
	    hal.console->println();
    }
    hal.console->printf_P(PSTR("[%u %.3f] "),
			  (unsigned)pin, v);
    pin = (pin+1) % 16;
    ch->set_pin(pin);
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
