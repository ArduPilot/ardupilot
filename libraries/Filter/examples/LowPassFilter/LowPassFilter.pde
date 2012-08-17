/*
 *       Example sketch to demonstrate use of LowPassFilter library.
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <Filter.h>                     // Filter library
#include <LowPassFilter.h>      // LowPassFilter class (inherits from Filter class)

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
FastSerialPort0(Serial);        // FTDI/console

// create a global instance of the class instead of local to avoid memory fragmentation
LowPassFilterInt16 low_pass_filter(0.02);  // simple low pass filter which applies 2% of new data to old data

// setup routine
void setup()
{
    // Open up a serial connection
    Serial.begin(115200);

    // introduction
    Serial.printf("ArduPilot LowPassFilter test ver 1.0\n\n");

    // Wait for the serial connection
    delay(500);
}

//Main loop where the action takes place
void loop()
{
    int16_t i;
    int16_t new_value;
    int16_t filtered_value;

    // reset value to 100.  If not reset the filter will start at the first value entered
    low_pass_filter.reset(100);

    for( i=0; i<210; i++ ) {

        // new data value
        new_value = 105;

        // output to user
        Serial.printf("applying: %d",(int)new_value);

        // apply new value and retrieved filtered result
        filtered_value = low_pass_filter.apply(new_value);

        // display results
        Serial.printf("\toutput: %d\n\n",(int)filtered_value);
    }
    delay(10000);
}


