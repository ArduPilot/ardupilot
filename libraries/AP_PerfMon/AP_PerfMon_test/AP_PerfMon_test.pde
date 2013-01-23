/*
  AP_PerfMon
  Code by Randy Mackay
*/

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_PerfMon.h>        // PerfMonitor library

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup()
{
  AP_PERFMON_REGISTER_NAME("setupA")

  hal.console->print_P(PSTR("Performance Monitor test v1.1\n"));
}

void loop()
{
    AP_PERFMON_REGISTER

    int16_t i = 0;

    for( i=0; i<10; i++ ) {
        testFn();
    }

    //AP_PerfMon::DisplayAndClear(5);
    AP_PerfMon::DisplayResults();
    AP_PerfMon::ClearAll();

    hal.scheduler->delay(10000);
}

void testFn()
{
    AP_PERFMON_REGISTER
    hal.scheduler->delay(10);
    testFn2();
    hal.scheduler->delay(10);
}

void testFn2()
{
    AP_PERFMON_REGISTER
    hal.scheduler->delay(10);
}
