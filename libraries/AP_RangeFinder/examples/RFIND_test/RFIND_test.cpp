/*
 *  RangeFinder test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static RangeFinder sonar{serial_manager, ROTATION_PITCH_270};

void setup()
{
    // print welcome message
    hal.console->printf("Range Finder library test\n");

    // setup for analog pin 13
    AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_PLI2C);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", -1.0f);
    AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 1.0f);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    sonar.init();
    hal.console->printf("RangeFinder: %d devices detected\n", sonar.num_sensors());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    sonar.update();

    bool had_data = false;
    for (uint8_t i=0; i<sonar.num_sensors(); i++) {
        AP_RangeFinder_Backend *sensor = sonar.get_backend(i);
        if (sensor == nullptr) {
            continue;
        }
        if (!sensor->has_data()) {
            continue;
        }
        hal.console->printf("All: device_%u type %d status %d distance_cm %d\n",
                            i,
                            (int)sensor->type(),
                            (int)sensor->status(),
                            sensor->distance_cm());
        had_data = true;
    }
    if (!had_data) {
        hal.console->printf("All: no data on any sensor\n");
    }

}
AP_HAL_MAIN();
