#include <AP_EEPROMB.h> 		// ArduPilot Mega RC Library

byte 	byte_value = 99;
int 	int_value = 30000;
long 	long_value = 123456789;
float 	float_value = .01234;

AP_EEPROMB ee;
void setup()
{
	Serial.begin(38400);
	delay(100);
	Serial.println("\nAP_EEPROMB test");

	Serial.println("AP_EEPROMB test");
	ee.write_byte(0, byte_value);
	ee.write_int(1, int_value);
	ee.write_long(3, long_value);
	ee.write_float(7, float_value);


	Serial.println(ee.read_byte(0), DEC);
	Serial.println(ee.read_int(1), DEC);
	Serial.println(ee.read_long(3), DEC);
	float e = ee.read_float(7);
	long y = e * 100;
	//Serial.println(e, 5);930 bytes
	
}

void loop()
{


}


// 2162 bytes
// 2216
