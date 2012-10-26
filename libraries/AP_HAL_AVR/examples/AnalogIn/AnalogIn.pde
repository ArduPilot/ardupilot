
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_HAL::AnalogSource* ch0;
AP_HAL::AnalogSource* ch1;
AP_HAL::AnalogSource* vdd;

void setup (void) {
    hal.uart0->printf_P(PSTR("Starting AP_HAL_AVR::AnalogIn test\r\n"));
    ch0 = hal.analogin->channel(0);
    ch1 = hal.analogin->channel(1);
    vdd = hal.analogin->channel(2);
}

void loop (void) {
    float meas_ch0 = ch0->read(); 
    float meas_ch1 = ch1->read();
    float meas_vdd = vdd->read();
    hal.uart0->printf_P(PSTR("read ch0: %f, ch1: %f, vdd: %f\r\n"),
            meas_ch0, meas_ch1, meas_vdd);
    hal.scheduler->delay(10);
}

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
