
#include <stdint.h>
#include <AP_Common.h>
#include <AP_Math.h>		// ArduPilot Mega Vector/Matrix math Library
#include <FastSerial.h>
#include <I2C.h>
#include <SPI.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_Baro.h> // ArduPilot Mega ADC Library

FastSerialPort0(Serial);

AP_Baro_MS5611 baro;
Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;

unsigned long timer;

void setup()
{  
	Serial.begin(115200, 128, 128);
	Serial.println("ArduPilot Mega MeasSense Barometer library test");

	delay(1000);

	isr_registry.init();
	scheduler.init(&isr_registry);

	pinMode(63, OUTPUT);
	digitalWrite(63, HIGH);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV32); // 500khz for debugging, increase later

	baro.init(&scheduler);
	timer = micros();
}

void loop()
{
	float tmp_float;
	float Altitude;
	
	if((micros()- timer) > 50000L){
		timer = micros();
		baro.read();
		unsigned long read_time = micros() - timer;
		if (!baro.healthy) {
			Serial.println("not healthy");
			return;
		}
		Serial.print("Pressure:");
		Serial.print(baro.get_pressure());
		Serial.print(" Temperature:");
		Serial.print(baro.get_temperature());
		Serial.print(" Altitude:");
		tmp_float = (baro.get_pressure() / 101325.0);
		tmp_float = pow(tmp_float, 0.190295);
		Altitude = 44330 * (1.0 - tmp_float);
		Serial.print(Altitude);
		Serial.printf(" t=%u", (unsigned)read_time);
		Serial.println();
	}
}
