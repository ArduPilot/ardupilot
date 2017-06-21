/*
 *       Example of AP_Parachute library.
 *       DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Notify/AP_Notify.h>
#include <StorageManager/StorageManager.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Relay
AP_Relay relay;

// Parachute
AP_Parachute parachute(relay);

void setup()
{
    hal.console->printf("AP_Parachute library test\n");
}

void loop()
{
    // print message to user
    hal.console->printf("this example tests compilation only");
    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();
