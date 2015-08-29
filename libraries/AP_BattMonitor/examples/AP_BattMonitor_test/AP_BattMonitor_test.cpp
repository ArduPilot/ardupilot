/*
 *       Example of AP_BattMonitor library
 *       Code by DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC/AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <DataFlash/DataFlash.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_BattMonitor battery_mon;
uint32_t timer;

void setup() {
    hal.console->println("Battery monitor library test");

    // set battery monitor to smbus
    battery_mon.set_monitoring(0, AP_BattMonitor::BattMonitor_TYPE_SMBUS);

    // initialise the battery monitor
    battery_mon.init();

    hal.scheduler->delay(1000);
    timer = hal.scheduler->millis();
}

void loop()
{
    static uint8_t counter; // counter to slow output to the user
    uint32_t now = hal.scheduler->millis();

    // call battery monitor at 10hz
    if((now - timer) > 100) {
        // update voltage and current readings
        battery_mon.read();

        // reset timer
        timer = now;

        // increment counter
        counter++;
    }

    // display output at 1hz
    if (counter >= 10) {
        counter = 0;
        hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f",
			    battery_mon.voltage(),
			    battery_mon.current_amps(),
                battery_mon.current_total_mah());
    }

    // delay 1ms
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
