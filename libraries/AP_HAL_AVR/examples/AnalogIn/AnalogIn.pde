
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_HAL::AnalogSource* ch0;
AP_HAL::AnalogSource* ch1;
AP_HAL::AnalogSource* vcc;

void setup (void) {
    hal.console->printf_P(PSTR("Starting AP_HAL_AVR::AnalogIn test\r\n"));
    ch0 = hal.analogin->channel(0);
    ch1 = hal.analogin->channel(1);
    vcc = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
}

void loop (void) {
    float meas_ch0 = ch0->read(); 
    float meas_ch1 = ch1->read();
    float meas_vcc = vcc->read();
    hal.console->printf_P(PSTR("read ch0: %f, ch1: %f, vcc: %f\r\n"),
            meas_ch0, meas_ch1, meas_vcc);
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
