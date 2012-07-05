/*
	Example sketch to demonstrate use of DerivativeFilter library.
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <Filter.h>
#include <DerivativeFilter.h>

#ifdef DESKTOP_BUILD
// all of this is needed to build with SITL
#include <DataFlash.h>
#include <APM_RC.h>
#include <GCS_MAVLink.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <Filter.h>
#include <SITL.h>
#include <I2C.h>
#include <SPI.h>
#include <AP_Declination.h>
Arduino_Mega_ISR_Registry isr_registry;
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL     compass;
SITL sitl;
BetterStream *mavlink_comm_0_port;
BetterStream *mavlink_comm_1_port;
mavlink_system_t mavlink_system;
#endif

FastSerialPort0(Serial);        // FTDI/console

DerivativeFilter<float,7> derivative;

// setup routine
void setup()
{
	// Open up a serial connection
	Serial.begin(115200);
	
	// introduction
	Serial.printf("ArduPilot DerivativeFilter test\n");
}

static float noise(void)
{
	return ((random() % 100)-50) * 0.001;
}

//Main loop where the action takes place
void loop()
{
	delay(50);
	float t = millis()*1.0e-3;
	float s = sin(t);
	s += noise();
	uint32_t t1 = micros();
	float output = derivative.apply(s, t1) * 1.0e6;
	uint32_t t2 = micros();
	Serial.printf("cos(t)=%.2f filter=%.2f tdiff=%u\n",
		      cos(t), output, t2-t1);
}


