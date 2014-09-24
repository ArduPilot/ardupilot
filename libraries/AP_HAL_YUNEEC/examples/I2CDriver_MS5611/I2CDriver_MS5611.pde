/*******************************************
*   Sample sketch that configures an HMC5883L 3 axis
*   magnetometer to continuous mode and reads back
*   the three axis of data.
*******************************************/

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <AP_Baro.h>
#include <Filter.h>             // Filter library
#include <utility/pinmap_typedef.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::DigitalSource *led;
static AP_Baro_MS5611 baro(&AP_Baro_MS5611::i2c);
static uint32_t timer;

void setup()
{
    hal.console->println("YUNEEC MS5611 Barometer library test");

    hal.scheduler->delay(1000);

    /* What's this for? */
    hal.gpio->pinMode(PC13, HAL_GPIO_OUTPUT);
    hal.gpio->write(PC13, 0);

    baro.init();
    baro.calibrate();

    timer = hal.scheduler->micros();
}

void loop()
{
    if((hal.scheduler->micros() - timer) > 100000UL) {
        timer = hal.scheduler->micros();
        baro.read();
        uint32_t read_time = hal.scheduler->micros() - timer;
        if (!baro.healthy) {
            hal.console->println("not healthy");
            return;
        }
        hal.console->print("Lock up count: ");
        hal.console->print(hal.i2c->lockup_count());
        hal.console->print(" Pressure:");
        hal.console->print(baro.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print(baro.get_temperature());
        hal.console->print(" Altitude:");
        hal.console->print(baro.get_altitude());
        hal.console->printf(" climb=%.2f t=%u samples=%u",
                      baro.get_climb_rate(),
                      (unsigned)read_time,
                      (unsigned)baro.get_pressure_samples());
        hal.console->println();
    }
}

AP_HAL_MAIN();
