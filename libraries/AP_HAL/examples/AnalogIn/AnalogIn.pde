
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::AnalogSource* ch0;
AP_HAL::AnalogSource* ch1;
AP_HAL::AnalogSource* ch2;
AP_HAL::AnalogSource* ch10;
AP_HAL::AnalogSource* ch11;
AP_HAL::AnalogSource* ch12;
AP_HAL::AnalogSource* ch13;
AP_HAL::AnalogSource* vcc;

void setup (void) {
    hal.console->printf_P(PSTR("Starting AP_HAL::AnalogIn test\r\n"));
    ch0 = hal.analogin->channel(0);
    ch1 = hal.analogin->channel(1);
    ch2 = hal.analogin->channel(2);
    ch10 = hal.analogin->channel(10);
    ch11 = hal.analogin->channel(11);
    ch12 = hal.analogin->channel(12);
    ch13 = hal.analogin->channel(13);
    vcc = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
}

void loop (void) {
    float meas_ch0  = ch0->read_average(); 
    float meas_ch1  = ch1->read_average();
    float meas_ch2  = ch2->read_average();
    float meas_ch10 = ch10->read_average();
    float meas_ch11 = ch11->read_average();
    float meas_ch12 = ch12->read_average();
    float meas_ch13 = ch13->read_average();
    float meas_vcc = vcc->read_average();
    hal.console->printf_P(PSTR("read ch0:%.1f ch1:%.1f ch2:%.1f ch10:%.1f ch11:%.1f ch12:%.1f ch13:%.1f vcc:%.1f\n"),
			  meas_ch0, 
			  meas_ch1, 
			  meas_ch2, 
			  meas_ch10, 
			  meas_ch11, 
			  meas_ch12, 
			  meas_ch13, 
			  meas_vcc);
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
