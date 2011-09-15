
/*
  Example of APM_ADC library.
  Code by Jordi Muñoz and Jose Julio. DIYDrones.com
*/

#include <FastSerial.h>
#include <AP_ADC.h> // ArduPilot Mega ADC Library

FastSerialPort0(Serial);        // FTDI/console

unsigned long timer;
AP_ADC_ADS7844 adc;

void setup()
{  
	Serial.begin(115200, 128, 128);
	Serial.println("ArduPilot Mega ADC library test");
	delay(1000);
	adc.Init();   // APM ADC initialization
	delay(1000);
	timer = millis();
}

static const uint8_t channel_map[6] = { 1, 2, 0, 4, 5, 6};
static uint16_t sin_count;
float v;
uint32_t last_usec = 0;

void loop()
{
	v = sin(millis());
	sin_count++;

	if ((millis() - timer) > 200) {
		uint16_t result[6];
		uint32_t deltat;
		uint16_t ch3;

		timer = millis();

		ch3 = adc.Ch(3);
		deltat = adc.Ch6(channel_map, result);

		Serial.printf("gx=%u gy=%u gz=%u ax=%u ay=%u az=%u gt=%u deltat=%lu sin_count=%u\n",
			      result[0], result[1], result[2],
			      result[3], result[4], result[5],
			      ch3, deltat, sin_count);
		sin_count = 0;
	}
}

