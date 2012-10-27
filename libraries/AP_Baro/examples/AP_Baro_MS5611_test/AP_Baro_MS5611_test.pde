
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

#include <Filter.h>
#include <AP_Baro.h>

AP_Baro_MS5611 baro;
static uint32_t timer;

void setup()
{
    hal.console->println("APM2 MS5611 Barometer library test");

    hal.scheduler->delay(1000);

    /* What's this for? */
    hal.gpio->pinMode(63, GPIO_OUTPUT);
    hal.gpio->write(63, 1);
    
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
        hal.console->print("Pressure:");
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

extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}

