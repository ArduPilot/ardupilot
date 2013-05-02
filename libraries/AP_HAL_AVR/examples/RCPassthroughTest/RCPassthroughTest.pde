
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

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


void multiwrite(AP_HAL::RCOutput* out, uint16_t* channels) {
    out->write(0, channels, 8);
    /* Upper channels duplicate lower channels*/
    out->write(8, channels, 8);
}

void individualwrite(AP_HAL::RCOutput* out, uint16_t* channels) {
    for (int ch = 0; ch < 8; ch++) {
        out->write(ch, channels[ch]); 
        /* Upper channels duplicate lower channels*/
        out->write(ch+8, channels[ch]);
    }
}

void loop (void) {
    static int ctr = 0;
    uint16_t channels[8];

    hal.gpio->write(27, 1);

    /* Cycle between using the individual read method
     * and the multi read method*/
    if (ctr < 500) {
        multiread(hal.rcin, channels);
    } else {
        individualread(hal.rcin, channels);
        if (ctr > 1000)  ctr = 0;
    }

    /* Cycle between individual output and multichannel output */
    if (ctr % 500 < 250) {
        multiwrite(hal.rcout, channels);
    } else {
        individualwrite(hal.rcout, channels);
    }

    hal.gpio->write(27, 0);
    hal.scheduler->delay(4);
    ctr++;
}

void setup (void) {
    hal.gpio->pinMode(27, GPIO_OUTPUT);
    hal.gpio->write(27, 0);
    hal.rcout->enable_mask(0x000000FF);

    /* Bottom 4 channels at 400hz (like on a quad) */
    hal.rcout->set_freq(0x0000000F, 400);
    for(int i = 0; i < 12; i++) {
        hal.console->printf_P(PSTR("rcout ch %d has frequency %d\r\n"),
                i, hal.rcout->get_freq(i));
    }
    /* Delay to let the user see the above printouts on the terminal */
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
