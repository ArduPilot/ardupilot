//
// Unit tests for the AP_Common code
//

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();
void loop();
void test_high_low_byte(void);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void test_high_low_byte(void)
{

    // test each value from 0 to 300
    for (uint16_t i = 0; i <= 300; i++) {
        uint8_t high = HIGHBYTE(i);
        uint8_t low = LOWBYTE(i);
        hal.console->printf("\ni:%u high:%u low:%u", (unsigned int)i, (unsigned int)high, (unsigned int)low);
    }

    // test values from 300 to 65400 at increments of 200
    for (uint16_t i = 301; i <= 65400; i += 200) {
        uint8_t high = HIGHBYTE(i);
        uint8_t low = LOWBYTE(i);
        hal.console->printf("\ni:%u high:%u low:%u", (unsigned int)i, (unsigned int)high, (unsigned int)low);
    }
}

/*
 *  euler angle tests
 */
void setup(void)
{
    hal.console->printf("AP_Common tests\n\n");

    test_high_low_byte();
}

void loop(void)
{
    // do nothing
}

AP_HAL_MAIN();
