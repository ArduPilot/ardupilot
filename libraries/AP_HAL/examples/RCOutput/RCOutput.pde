
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

void setup (void) 
{
    hal.console->printf_P(PSTR("Starting AP_HAL::RCOutput test\r\n"));
    uint8_t i;

    hal.console->printf_P(PSTR("Testing 14 channels 1050+i*50\n"));
    for (i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, 1050 + i*50);
    }
    hal.rcout->set_freq(0xFF, 490);
}

void loop (void) 
{
    uint8_t i;
    for (i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, 1050 + i*50);
    }
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
