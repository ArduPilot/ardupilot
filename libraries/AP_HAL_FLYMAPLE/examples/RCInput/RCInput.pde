
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void multiread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* Multi-channel read method: */
    uint8_t valid;
    valid = in->read(channels, 8);
    hal.console->printf_P(
            PSTR("multi      read %d: %d %d %d %d %d %d %d %d\r\n"),
            (int) valid, 
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]);
}

void individualread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* individual channel read method: */
    uint8_t valid;
    valid = in->valid_channels();
    for (int i = 0; i < 8; i++) {
        channels[i] = in->read(i);
    }
    hal.console->printf_P(
            PSTR("individual read %d: %d %d %d %d %d %d %d %d\r\n"),
            (int) valid, 
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]);
}

void loop (void) {
    static int ctr = 0;
    uint16_t channels[8];

    hal.gpio->write(13, 1);

    /* Cycle between using the individual read method
     * and the multi read method*/
    if (ctr < 500) {
        multiread(hal.rcin, channels);
    } else {
        individualread(hal.rcin, channels);
        if (ctr > 1000)  ctr = 0;
    }

    hal.gpio->write(13, 0);
    hal.scheduler->delay(50);
    ctr++;
}

void setup (void) {
    hal.gpio->pinMode(13, GPIO_OUTPUT);
    hal.gpio->write(13, 0);
}

AP_HAL_MAIN();
