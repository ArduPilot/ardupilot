/*
	Example of Filter library.
	Code by Randy Mackay and Jason Short. DIYDrones.com
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>		// ArduPilot Mega Vector/Matrix math Library
#include <Filter.h>			// Filter library
#include <ModeFilter.h>		// ModeFilter library (inherits from Filter class)
#include <SumFilter.h>

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
FastSerialPort0(Serial);        // FTDI/console

//typedef ModeFilter<int16_t> IntModeFilter;
//typedef SumFilter<int16_t> IntSumFilter;

int16_t rangevalue[] = {31000, 31000, 50, 55, 60, 55, 10, 0, 31000};

// create a global instance of the class instead of local to avoid memory fragmentation
ModeFilterInt16 mfilter(5,2);  // buffer of 5 values, result will be from buffer element 2 (ie. the 3rd element which is the middle)
//SumFilterInt16 mfilter(5);

//Function to print contents of a filter
void printFilter(Filter<int16_t>& filter)
{
	for(int8_t i=0; i < filter.filter_size; i++)
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

//Main loop where the action takes place
void loop()
{
    int8_t i = 0;
	int16_t filtered_value;

	while( i < 9 ) {

		// output to user
		Serial.printf("applying: %d\n",(int)rangevalue[i]);

		// display original
		Serial.printf("before: ");
		printFilter(mfilter);

		// apply new value and retrieved filtered result
		filtered_value = mfilter.apply(rangevalue[i]);

		// display results
		Serial.printf("after: ");
		printFilter(mfilter);
		Serial.printf("The filtered value is: %d\n\n",(int)filtered_value);

		i++;
	}
	delay(100000);
}


