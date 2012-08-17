/*
 *       Example of Filter library.
 *       Code by Randy Mackay and Jason Short. DIYDrones.com
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <Filter.h>                     // Filter library
#include <ModeFilter.h>         // ModeFilter class (inherits from Filter class)
#include <AverageFilter.h>      // AverageFilter class (inherits from Filter class)

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
FastSerialPort0(Serial);        // FTDI/console

int16_t rangevalue[] = {31000, 31000, 50, 55, 60, 55, 10, 0, 31000};

// create a global instance of the class instead of local to avoid memory fragmentation
ModeFilterInt16_Size5 mfilter(2);  // buffer of 5 values, result will be from buffer element 2 (ie. the 3rd element which is the middle)
//AverageFilterInt16_Size5 mfilter;  // buffer of 5 values.  result will be average of these 5

AverageFilterUInt16_Size4 _temp_filter;

// Function to print contents of a filter
// we need to ues FilterWithBuffer class because we want to access the individual elements
void printFilter(FilterWithBufferInt16_Size5& filter)
{
    for(uint8_t i=0; i < filter.get_filter_size(); i++)
    {
        Serial.printf("%d ",(int)filter.samples[i]);
    }
    Serial.println();
}

void setup()
{
    // Open up a serial connection
    Serial.begin(115200);

    // introduction
    Serial.printf("ArduPilot ModeFilter library test ver 1.0\n\n");

    // Wait for the serial connection
    delay(500);
}

// Read Raw Temperature values
void ReadTemp()
{
    static uint8_t next_num = 0;
    static int32_t RawTemp = 0;
    uint8_t buf[2];
    uint16_t _temp_sensor;

    next_num++;
    buf[0] = next_num;     //next_num;
    buf[1] = 0xFF;

    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    RawTemp = _temp_filter.apply(_temp_sensor);

    Serial.printf("RT: %ld\n",RawTemp);
}

//Main loop where the action takes place
void loop()
{
    uint8_t i = 0;
    int16_t filtered_value;

    int16_t j;

    for(j=0; j<0xFF; j++ ) {
        ReadTemp();
    }
    /*while( i < 9 ) {
     *
     *       // output to user
     *       Serial.printf("applying: %d\n",(int)rangevalue[i]);
     *
     *       // display original
     *       Serial.printf("before: ");
     *       printFilter(mfilter);
     *
     *       // apply new value and retrieved filtered result
     *       filtered_value = mfilter.apply(rangevalue[i]);
     *
     *       // display results
     *       Serial.printf("after: ");
     *       printFilter(mfilter);
     *       Serial.printf("The filtered value is: %d\n\n",(int)filtered_value);
     *
     *       i++;
     *  }*/
    delay(10000);
}


