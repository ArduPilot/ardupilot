/*
	Example of GPS MTK library.
	Code by Jordi Munoz and Jose Julio. DIYDrones.com

	Works with Ardupilot Mega Hardware (GPS on Serial Port1)
	and with standard ATMega168 and ATMega328 on Serial Port 0
*/

#include <AP_GPS_MTK.h> // UBLOX GPS Library

AP_GPS_MTK gps;
#define T6 1000000
#define T7 10000000

void setup()
{
	Serial.begin(38400);
	Serial.println("GPS MTK library test");
	gps.init();	 // GPS Initialization
	delay(1000);
}
void loop()
{
	delay(20);
	gps.update();
	if (gps.new_data){
		Serial.print("gps:");
		Serial.print(" Lat:");
		Serial.print((float)gps.lattitude / T7, DEC);
		Serial.print(" Lon:");
		Serial.print((float)gps.longitude / T7, DEC);
		Serial.print(" Alt:");
		Serial.print((float)gps.altitude / 100.0, DEC);
		Serial.print(" GSP:");
		Serial.print(gps.ground_speed / 100.0);
		Serial.print(" COG:");
		Serial.print(gps.ground_course / 100.0, DEC);
		Serial.print(" SAT:");
		Serial.print(gps.num_sats, DEC);
		Serial.print(" FIX:");
		Serial.print(gps.fix, DEC);
		Serial.print(" TIM:");
		Serial.print(gps.time, DEC);
		Serial.println();
		gps.new_data = 0; // We have readed the data
		}
}

