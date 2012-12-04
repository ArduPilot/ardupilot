
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

void multiread(AP_HAL::RCInput* in) {
    /* Multi-channel read method: */
    uint16_t channels[8];
    uint8_t valid;
    valid = in->read(channels, 8);
    hal.console->printf_P(
            PSTR("multi      read %d: %d %d %d %d %d %d %d %d\r\n"),
            (int) valid, 
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]);
}

void individualread(AP_HAL::RCInput* in) {
    /* individual channel read method: */
    uint8_t valid;
    uint16_t channels[8];
    valid = in->valid();
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

    hal.gpio->write(27, 1);
    
    /* Cycle between using the individual read method
     * and the multi read method*/
    if (ctr < 500) {
        multiread(hal.rcin);
    } else {
        individualread(hal.rcin);
        if (ctr > 1000)  ctr = 0;
    }
    ctr++;

    hal.gpio->write(27, 0);
    hal.scheduler->delay(2);
}

void setup (void) {
    hal.console->printf_P(PSTR("reading rc in:"));
    hal.gpio->pinMode(27, GPIO_OUTPUT);
    hal.gpio->write(27, 0);
}

AP_HAL_MAIN();
