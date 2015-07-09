/*
  generic Baro driver test
 */
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Rally.h>
#include <AP_NavEKF.h>
#include <AP_Scheduler.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_Baro barometer;

static uint32_t timer;
static uint8_t counter;

void setup()
{
    hal.console->println("Barometer library test");

    hal.scheduler->delay(1000);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // disable CS on MPU6000
    hal.gpio->pinMode(63, HAL_GPIO_OUTPUT);
    hal.gpio->write(63, 1);
#endif

    barometer.init();
    barometer.calibrate();

    timer = hal.scheduler->micros();
}

void loop()
{
    // run accumulate() at 50Hz and update() at 10Hz
    if((hal.scheduler->micros() - timer) > 20*1000UL) {
        timer = hal.scheduler->micros();
        barometer.accumulate();
        if (counter++ < 5) {
            return;
        }
        counter = 0;
        barometer.update();
        uint32_t read_time = hal.scheduler->micros() - timer;
        float alt = barometer.get_altitude();
        if (!barometer.healthy()) {
            hal.console->println("not healthy");
            return;
        }
        hal.console->printf("Pressure: %.2f Pa "
                            "Temperature: %.2f ºC "
                            "Altitude: %.2f m "
                            "climb:%.2f m/s"
                            "t:%u usec\n", barometer.get_pressure(),
                            barometer.get_temperature(), alt,
                            barometer.get_climb_rate(),
                            (unsigned)read_time);
    }
}

AP_HAL_MAIN();
