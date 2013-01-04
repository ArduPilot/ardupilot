
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

AP_Baro_PX4 baro;
static uint32_t timer;

void setup()
{
    hal.console->println("PX4 Barometer library test");

    baro.init();
    baro.calibrate();

    timer = hal.scheduler->micros();
}

void loop()
{
	hal.scheduler->delay(100);
        baro.read();

        if (!baro.healthy) {
            hal.console->println("not healthy");
            return;
        }
        hal.console->print("Pressure:");
        hal.console->print(baro.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print(baro.get_temperature());
        hal.console->print(" Altitude:");
        hal.console->print(baro.get_altitude());
        hal.console->printf(" climb=%.2f samples=%u",
                      baro.get_climb_rate(),
                      (unsigned)baro.get_pressure_samples());
        hal.console->println();
}

#else // Non-PX4
#warning AP_Baro_PX4_test is PX4 specific
void setup () {}
void loop () {}
#endif

AP_HAL_MAIN();
