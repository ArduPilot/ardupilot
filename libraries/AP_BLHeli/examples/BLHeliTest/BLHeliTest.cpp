/*
  example implementing MSP and BLHeli passthrough protocol in ArduPilot

  With thanks to betaflight for a great reference implementation
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BLHeli/AP_BLHeli.h>

extern const AP_HAL::HAL& hal;

void setup();
void loop();

static AP_BLHeli esc_serial;

void setup(void) {
    hal.console->begin(115200);
    hal.scheduler->delay(1000);
    hal.console->printf("ESCSerial Starting\n");

    hal.uartA->begin(115200, 1024, 1024);
    hal.uartC->begin(115200, 256, 256);

    esc_serial.init(hal.uartA, hal.uartC);
    
    hal.rcout->init();
    hal.rcout->set_esc_scaling(1000, 2000);
    hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_NORMAL);
    hal.rcout->set_freq(0xF, 400);
    hal.rcout->cork();
    for (uint8_t i=0; i<4; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, 1000);
    }
    hal.rcout->push();
}

void loop(void)
{
    int16_t b = hal.uartA->read();
    if (b == -1) {
        return;
    }
    esc_serial.process_input(b);
}

AP_HAL_MAIN();
