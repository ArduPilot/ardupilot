/*
 *       Example of Filter library.
 *       Code by Randy Mackay and Jason Short. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_Param.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <Filter.h>                     // Filter library
#include <ModeFilter.h>         // ModeFilter class (inherits from Filter class)
#include <AverageFilter.h>      // AverageFilter class (inherits from Filter class)
#include <AP_Buffer.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

int16_t rangevalue[] = {31000, 31000, 50, 55, 60, 55, 10, 0, 31000};

// create a global instance of the class instead of local to avoid memory fragmentation
ModeFilterInt16_Size5 mfilter(2);  // buffer of 5 values, result will be from buffer element 2 (ie. the 3rd element which is the middle)
//AverageFilterInt16_Size5 mfilter;  // buffer of 5 values.  result will be average of these 5

AverageFilterUInt16_Size4 _temp_filter;

void setup()
{
    // Open up a serial connection
    hal.uart0->begin(115200);

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
    buf[0] = next_num;     //next_num;
    buf[1] = 0xFF;

    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    raw_temp = _temp_filter.apply(_temp_sensor);

    hal.console->printf("RT: %ld\n", raw_temp);
}

//Main loop where the action takes place
void loop()
{
    uint8_t i = 0;
    int16_t filtered_value;

    int16_t j;

    for(j=0; j<0xFF; j++ ) {
        readTemp();
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(10000);
}

AP_HAL_MAIN();
