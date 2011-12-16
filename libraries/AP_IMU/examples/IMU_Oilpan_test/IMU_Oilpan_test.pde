// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_IMU_Oilpan driver.
//

#include <FastSerial.h>
#include <SPI.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_IMU.h>
#include <AP_ADC.h>
#include <AP_Math.h>
#include <AP_Common.h>

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerPeriodicProcess  adc_scheduler;

AP_ADC_ADS7844	adc;
AP_InertialSensor_Oilpan oilpan_ins(&adc);
AP_IMU_INS imu(&oilpan_ins,0);

void setup(void)
{
	Serial.begin(115200);
	Serial.println("Doing IMU startup...");

    isr_registry.init();
    adc_scheduler.init(&isr_registry);

    /* Should also call ins.init and adc.init */
	imu.init(IMU::COLD_START, delay, &adc_scheduler);
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
