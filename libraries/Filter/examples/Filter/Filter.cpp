/*
 *       Example of Filter library.
 *       Code by Randy Mackay and Jason Short. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <Filter/Filter.h>                     // Filter library
#include <Filter/ModeFilter.h>         // ModeFilter class (inherits from Filter class)
#include <Filter/AverageFilter.h>      // AverageFilter class (inherits from Filter class)

void setup();
void loop();
void readTemp();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

int16_t rangevalue[] = {31000, 31000, 50, 55, 60, 55, 10, 0, 31000};

// create a global instance of the class instead of local to avoid memory fragmentation
ModeFilterInt16_Size5 mfilter(2);  // buffer of 5 values, result will be from buffer element 2 (ie. the 3rd element which is the middle)
//AverageFilterInt16_Size5 mfilter;  // buffer of 5 values.  result will be average of these 5

AverageFilterUInt16_Size4 _temp_filter;

butter50hz8_0 butter;

void setup()
{
    // introduction
    hal.console->printf("ArduPilot ModeFilter library test ver 1.0\n\n");

    // Wait for the serial connection
    hal.scheduler->delay(500);
}

// Read Raw Temperature values
void readTemp()
{
    static uint8_t next_num = 0;
    static int32_t raw_temp = 0;
    uint8_t buf[2];
    uint16_t _temp_sensor;

    next_num++;
    buf[0] = next_num;  //next_num;
    buf[1] = 0xFF;

    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    raw_temp = _temp_filter.apply(_temp_sensor);

    // use a butter filter on the result, just so we have a
    // butterworth filter example
    butter.filter(raw_temp);

    hal.console->printf("RT: %lu\n", (unsigned long)raw_temp);
}

// Main loop where the action takes place
void loop()
{
    for (uint8_t j = 0; j < 0xFF; j++) {
        readTemp();
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
