//
// Test for AP_GPS_AUTO
//

#include <FastSerial.h>
#include <AP_GPS.h>
#include <stdlib.h>

FastSerialPort0(Serial);
FastSerialPort1(Serial1);

GPS	    *gps;
AP_GPS_Auto GPS(&Serial1, &gps);

#define T6 1000000
#define T7 10000000

void * operator new(size_t size) 
{
	return(calloc(size, 1));
}

void setup()
{
	Serial.begin(38400);
	Serial1.begin(38400);

	Serial.println("GPS AUTO library test");
	gps = &GPS;
	gps->init();
	delay(1000);
}
void loop()
{
	delay(20);
	gps->update();
	if (gps->new_data){
		Serial.print("gps:");
		Serial.print(" Lat:");
		Serial.print((float)gps->latitude / T7, DEC);
		Serial.print(" Lon:");
		Serial.print((float)gps->longitude / T7, DEC);
		Serial.print(" Alt:");
		Serial.print((float)gps->altitude / 100.0, DEC);
		Serial.print(" GSP:");
		Serial.print(gps->ground_speed / 100.0);
		Serial.print(" COG:");
		Serial.print(gps->ground_course / 100.0, DEC);
		Serial.print(" SAT:");
		Serial.print(gps->num_sats, DEC);
		Serial.print(" FIX:");
		Serial.print(gps->fix, DEC);
		Serial.print(" TIM:");
		Serial.print(gps->time, DEC);
		Serial.println();
		gps->new_data = 0;
	}
}

