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
#include <AP_Math.h>
#include <AP_Common.h>

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  mpu_scheduler;

AP_InertialSensor_MPU6000 mpu6000( 53 ); /* chip select is pin 53 */
AP_IMU_INS imu(&mpu6000, 0); /* second arg is for Parameters. Can we leave it null?*/

#define A_LED_PIN        27
#define C_LED_PIN        25
#define LED_ON           LOW
#define LED_OFF          HIGH

static void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

void setup(void)
{
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

	Serial.begin(115200);
	Serial.println("Doing IMU startup...");

    isr_registry.init();
    mpu_scheduler.init(&isr_registry);

	imu.init(IMU::COLD_START, delay, flash_leds, &mpu_scheduler);
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
