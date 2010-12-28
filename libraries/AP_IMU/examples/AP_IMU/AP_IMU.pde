// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_IMU driver.
//

#include <FastSerial.h>
#include <AP_IMU.h>
#include <AP_ADC.h>
#include <AP_Math.h>
#include <AP_Common.h>

FastSerialPort(Serial, 0);

AP_ADC_ADS7844	adc;
AP_IMU_Oilpan	imu(&adc, 0);	// disable warm-start for now

void setup(void)
{
	Serial.begin(38400);
	Serial.println("Doing IMU startup...");
	adc.Init();
	imu.init(IMU::COLD_START);
}

void loop(void)
{
	Vector3f	accel;
	Vector3f	gyro;

	delay(1000);
	imu.update();
	accel = imu.get_accel();
	gyro = imu.get_gyro();

	Serial.printf("AX: 0x%4.4f  AY: 0x%4.4f  AZ: 0x%4.4f  GX: 0x%4.4f  GY: 0x%4.4f  GZ: 0x%4.4f\n",
				  accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
}
