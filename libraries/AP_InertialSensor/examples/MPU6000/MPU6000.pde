// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_InertialSensor MPU6000 driver.
//

#include <FastSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;
AP_InertialSensor_MPU6000 ins( 53 ); /* chip select is pin 53 */

void setup(void)
{
	Serial.begin(115200);
	Serial.println("Doing INS startup...");

	Wire.begin();
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHZ SPI rate

    isr_registry.init();
    scheduler.init(&isr_registry);

	// we need to stop the barometer from holding the SPI bus
	pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);

	ins.init(&scheduler);
}

void loop(void)
{
	float accel[3];
	float gyro[3];
	float temperature;

	delay(20);
	ins.update();
	ins.get_gyros(gyro);
	ins.get_accels(accel);
	temperature = ins.temperature();

	Serial.printf("AX: %f  AY: %f  AZ: %f  GX: %f  GY: %f  GZ: %f T=%f\n",
				  accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temperature);
}
