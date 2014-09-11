
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>

#include <utility/pinmap_typedef.h>

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
    bool valid;
    valid = in->new_input();
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
    out->write(0, channels, 6);
}

void individualwrite(AP_HAL::RCOutput* out, uint16_t* channels) {
    for (int ch = 0; ch < 6; ch++) {
        out->write(ch, channels[ch]); 
    }
}

static uint16_t channels[6] = {0};

void loop (void) {
    static int ctr = 0;
    hal.gpio->write(PC13, 0);

    /* Cycle between using the individual read method
     * and the multi read method*/
    if (ctr < 500) {
        multiread(hal.rcin, channels);
    } else {
        individualread(hal.rcin, channels);
        if (ctr > 1000)  ctr = 0;
    }

    multiwrite(hal.rcout, channels);

    /* Cycle between individual output and multichannel output */
    if (ctr % 500 < 250) {
        multiwrite(hal.rcout, channels);
    } else {
        individualwrite(hal.rcout, channels);
    }

    hal.gpio->write(PC13, 1);
    hal.scheduler->delay(4);
    ctr++;
}

void setup (void) {
//    hal.scheduler->delay(5000);
    hal.gpio->pinMode(PC13, HAL_GPIO_OUTPUT);
    hal.gpio->write(PC13, 1);
    for (uint8_t i=0; i<6; i++) {
        hal.rcout->enable_ch(i);
    }

    multiwrite(hal.rcout, channels);

    /* Bottom 4 channels at 400hz (like on a quad) */
    hal.rcout->set_freq(0x0000000F, 400);
    for(int i = 0; i < 6; i++) {
        hal.console->printf_P(PSTR("rcout ch %d has frequency %d\r\n"),
                i, hal.rcout->get_freq(i));
    }
    /* Delay to let the user see the above printouts on the terminal */
    hal.scheduler->delay(3000);
}

AP_HAL_MAIN();
