/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Common code
//

#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_Math/AP_Math.h>
#include <StorageManager/StorageManager.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void test_high_low_byte(void)
{
    uint16_t i;
    uint8_t high, low;

    // test each value from 0 to 300
    for (i=0; i<=300; i++) {
        high = HIGHBYTE(i);
        low = LOWBYTE(i);
        hal.console->printf_P(PSTR("\ni:%u high:%u low:%u"),(unsigned int)i, (unsigned int)high, (unsigned int)low);
    }

    // test values from 300 to 65400 at increments of 200
    for (i=301; i<=65400; i+=200) {
        high = HIGHBYTE(i);
        low = LOWBYTE(i);
        hal.console->printf_P(PSTR("\ni:%u high:%u low:%u"),(unsigned int)i, (unsigned int)high, (unsigned int)low);
    }
}

/*
 *  euler angle tests
 */
void setup(void)
{
    hal.console->println("AP_Common tests\n");

    test_high_low_byte();
}

void loop(void)
{
    // do nothing
}

AP_HAL_MAIN();
