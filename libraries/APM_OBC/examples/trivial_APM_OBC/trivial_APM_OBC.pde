
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <APM_OBC.h>

#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup () {
    hal.console->println_P(PSTR("Unit test for APM_OBC. This sketch"
                "has no functionality, it only tests build."));
}
void loop () {}

AP_HAL_MAIN();
