/*
 *       Example of AP_BattMonitor library
 *       Code by DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_BattMonitor.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_BattMonitor battery_mon;
uint32_t timer;

void setup() {
    hal.console->println("Battery monitor library test");

    // initialise the battery monitor
    battery_mon.init();
    battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);

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
