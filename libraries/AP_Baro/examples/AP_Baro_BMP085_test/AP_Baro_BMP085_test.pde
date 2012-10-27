/*
 *       Example of APM_BMP085 (absolute pressure sensor) library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */


#include <FastSerial.h>
#include <AP_Common.h>
#include <I2C.h>
#include <SPI.h>
#include <Filter.h>
#include <AP_Baro.h> // ArduPilot Mega BMP085 Library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <math.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <Filter.h>
#include <AP_Baro.h>

#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;

AP_Baro_BMP085 bmp085(0);

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

    if((hal.scheduler->micros()- timer) > 50000L) {
        timer = hal.scheduler->micros();
        bmp085.read();
        uint32_t read_time = hal.scheduler->micros() - timer;
        if (! bmp085.healthy) {
            hal.console->println("not healthy");
            return;
        }
        hal.console->print("Pressure:");
        hal.console->print( bmp085.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print( bmp085.get_temperature());
        hal.console->print(" Altitude:");
        tmp_float = ( bmp085.get_pressure() / 101325.0);
        tmp_float = pow(tmp_float, 0.190295);
        float alt = 44330.0 * (1.0 - tmp_float);
        hal.console->print(alt);
        hal.console->printf(" t=%lu", read_time);
        hal.console->println();
    }
}

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}

