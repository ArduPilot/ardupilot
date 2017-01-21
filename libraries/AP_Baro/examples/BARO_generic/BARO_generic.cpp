/*
  generic Baro driver test
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Baro barometer;

static uint32_t timer;
static uint8_t counter;

void setup()
{
    hal.console->printf("Barometer library test\n");

    AP_BoardConfig{}.init();

    hal.scheduler->delay(1000);

    barometer.init();
    barometer.calibrate();

    timer = AP_HAL::micros();
}

void loop()
{
    // run accumulate() at 50Hz and update() at 10Hz
    if((AP_HAL::micros() - timer) > 20*1000UL) {
        timer = AP_HAL::micros();
        barometer.accumulate();
        if (counter++ < 5) {
            return;
        }
        counter = 0;
        barometer.update();
        uint32_t read_time = AP_HAL::micros() - timer;
        float alt = barometer.get_altitude();
        if (!barometer.healthy()) {
            hal.console->printf("not healthy\n");
            return;
        }
        hal.console->printf("Pressure:");
        hal.console->printf(barometer.get_pressure());
        hal.console->printf(" Temperature:");
        hal.console->printf(barometer.get_temperature());
        hal.console->printf(" Altitude:");
        hal.console->printf(alt);
        hal.console->printf(" climb=%.2f t=%u",
                            barometer.get_climb_rate(),
                            (unsigned)read_time);
        hal.console->print("\n");
    } else {
        hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
