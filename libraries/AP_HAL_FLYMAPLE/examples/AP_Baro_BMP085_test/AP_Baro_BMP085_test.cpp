/*
 *       Example of APM_BMP085 (absolute pressure sensor) library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */


#include <AP_Common/AP_Common.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <math.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Buffer/AP_Buffer.h>
#include <AP_Notify/AP_Notify.h>
#include <Filter/Filter.h>
#include <AP_Baro/AP_Baro.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>

#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_Baro_BMP085 bmp085;

uint32_t timer;

void setup()
{
    hal.console->println("ArduPilot Mega BMP085 library test");
    hal.console->println("Initialising barometer...");

    hal.scheduler->delay(100);

    if (!bmp085.init()) {
        hal.console->println("Barometer initialisation FAILED\n");
    }
    hal.console->println("initialisation complete.");
    hal.scheduler->delay(1000);
    timer = hal.scheduler->micros();
}

void loop()
{
    float tmp_float;
    static uint32_t last_print;

    // accumulate values at 100Hz
    if ((hal.scheduler->micros()- timer) > 20000L) {
	    bmp085.accumulate();
	    timer = hal.scheduler->micros();
    }

    // print at 10Hz
    if ((hal.scheduler->millis()- last_print) >= 100) {
	uint32_t start = hal.scheduler->micros();
        last_print = hal.scheduler->millis();
        bmp085.read();
        uint32_t read_time = hal.scheduler->micros() - start;
        if (! bmp085.healthy) {
            hal.console->println("not healthy");
            return;
        }
        hal.console->print("Pressure:");
        hal.console->print( bmp085.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print( bmp085.get_temperature());
        hal.console->print(" Altitude:");
        tmp_float = ( bmp085.get_pressure() / 101325.0f);
        tmp_float = pow(tmp_float, 0.190295f);
        float alt = 44330.0f * (1.0f - tmp_float);
        hal.console->print(alt);
        hal.console->printf(" t=%lu", 
			    read_time);
        hal.console->println();
    }
}

AP_HAL_MAIN();
