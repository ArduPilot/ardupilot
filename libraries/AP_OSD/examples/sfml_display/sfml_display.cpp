/**
 * Usage
 * 
 * Configure
 * ./waf configure --board=sitl --osd --osd-fonts --sitl-osd --enable-sfml --debug
 * 
 * Build
 * ./waf build --target examples/sfml_display
 * 
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OSD/AP_OSD.h>

#include <chrono>
#include <thread>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Notify notify;
static AP_OSD osd;

void setup()
{
    hal.console->begin(115200);
    hal.console->printf("SFML Display\n\n");

    /// \note require notify for the singleton and semaphore,
    ///       but don't initialise as we don't have I2C devices.
    // notify.init();

    osd.init();

   return;
}

void loop()
{
    // update display
    osd.update();

    /// \note hal.scheduler tries to update a FDM, even for a simple case?
    // hal.scheduler->delay(20);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

AP_HAL_MAIN();
