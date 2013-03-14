
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
AP_HAL::AnalogSource* ch3;
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
    ch3 = hal.analogin->channel(3);
    ch10 = hal.analogin->channel(10);
    ch11 = hal.analogin->channel(11);
    ch12 = hal.analogin->channel(12);
    ch13 = hal.analogin->channel(13);
    vcc = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
}

void loop (void) {
    float meas_ch0  = ch0->voltage_average(); 
    float meas_ch1  = ch1->voltage_average();
    float meas_ch2  = ch2->voltage_average();
    float meas_ch3  = ch3->voltage_average();
    float meas_ch10 = ch10->voltage_average();
    float meas_ch11 = ch11->voltage_average();
    float meas_ch12 = ch12->voltage_average();
    float meas_ch13 = ch13->voltage_average();
    float meas_vcc = vcc->read_average() * 0.001;
    hal.console->printf_P(PSTR("Voltage ch0:%.2f ch1:%.2f ch2:%.2f ch3:%.2f ch10:%.2f ch11:%.2f ch12:%.2f ch13:%.2f vcc:%.2f\n"),
			  meas_ch0, 
			  meas_ch1, 
			  meas_ch2, 
			  meas_ch3, 
			  meas_ch10, 
			  meas_ch11, 
			  meas_ch12, 
			  meas_ch13, 
			  meas_vcc);
    hal.scheduler->delay(200);
}

AP_HAL_MAIN();
