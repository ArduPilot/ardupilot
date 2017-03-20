/*
  generic Baro driver test
 */

#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_Baro barometer;

static uint32_t timer;
static uint8_t counter;

void setup();
void loop();

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
    if (!hal.console->is_initialized()) {
        return;
    }

    // run accumulate() at 50Hz and update() at 10Hz
    if ((AP_HAL::micros() - timer) > 20 * 1000UL) {
        timer = AP_HAL::micros();
        barometer.accumulate();
        if (counter++ < 5) {
            return;
        }
        counter = 0;
        barometer.update();
        uint32_t read_time = AP_HAL::micros() - timer;
        if (!barometer.healthy()) {
            hal.console->printf("not healthy\n");
            return;
        }
        hal.console->printf(" Pressure: %.2f Pa\n"
                            " Temperature: %.2f degC\n"
                            " Relative Altitude: %.2f m\n"
                            " climb=%.2f m/s\n"
                            " Read + update time: %u usec\n"
                            "\n",
                            (double)barometer.get_pressure(),
                            (double)barometer.get_temperature(),
                            (double)barometer.get_altitude(),
                            (double)barometer.get_climb_rate(),
                            read_time);
    } else {
        hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
