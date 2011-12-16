// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_IMU driver.
//

#include <FastSerial.h>
#include <AP_IMU.h>
#include <AP_IMU_MPU6000.h>             // Experimental MPU6000 IMU library
#include <AP_PeriodicProcess.h>         // Parent header of Timer
                                        // (only included for makefile libpath to work)
#include <AP_TimerProcess.h>            // TimerProcess is the scheduler for MPU6000 reads.
#include <AP_ADC.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <SPI.h>

FastSerialPort(Serial, 0);

#ifndef CONFIG_MPU6000_CHIP_SELECT_PIN
#  define CONFIG_MPU6000_CHIP_SELECT_PIN 53
#endif

AP_IMU_MPU6000 imu(140,
				   CONFIG_MPU6000_CHIP_SELECT_PIN);
AP_TimerProcess timer_scheduler;
AP_ADC_ADS7844	adc;

void setup(void)
{
	Serial.begin(115200);
	Serial.println("Doing IMU startup...");
	timer_scheduler.init();
	Serial.println("done timer init");
	adc.Init(&timer_scheduler);
	Serial.println("done adc init");
	imu.init(IMU::COLD_START, delay, &timer_scheduler);
	Serial.println("done IMU init");
	delay(1000);
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
