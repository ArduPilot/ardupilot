/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Common code
//

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void test_high_low_byte(void)
{
    uint16_t i;
    uint8_t high, low;

    // test each value from 0 to 300
    for (i=0; i<=300; i++) {
        high = HIGHBYTE(i);
        low = LOWBYTE(i);
        hal.console->printf("\ni:%u high:%u low:%u",(unsigned int)i, (unsigned int)high, (unsigned int)low);
    }

    // test values from 300 to 65400 at increments of 200
    for (i=301; i<=65400; i+=200) {
        high = HIGHBYTE(i);
        low = LOWBYTE(i);
        hal.console->printf("\ni:%u high:%u low:%u",(unsigned int)i, (unsigned int)high, (unsigned int)low);
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
