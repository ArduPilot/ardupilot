/*
 *       Example of AP_BattMonitor library
 *       Code by DIYDrones.com
 */

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_BattMonitor battery_mon;
uint32_t timer;

void setup() {
    hal.console->println("Battery monitor library test");

    // set battery monitor to smbus
    battery_mon.set_monitoring(0, AP_BattMonitor::BattMonitor_TYPE_SMBUS);

    // initialise the battery monitor
    battery_mon.init();

    hal.scheduler->delay(1000);
    timer = AP_HAL::millis();
}

void loop()
{
    static uint8_t counter; // counter to slow output to the user
    uint32_t now = AP_HAL::millis();

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
