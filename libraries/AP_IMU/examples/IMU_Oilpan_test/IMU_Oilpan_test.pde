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

#define A_LED_PIN        27
#define C_LED_PIN        25
#define LED_ON           LOW
#define LED_OFF          HIGH

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess adc_scheduler;

AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan oilpan_ins(&adc);
AP_IMU_INS imu(&oilpan_ins,0);

static void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on ? LED_OFF : LED_ON);
    digitalWrite(C_LED_PIN, on ? LED_ON : LED_OFF);
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Doing IMU startup...");

    isr_registry.init();
    adc_scheduler.init(&isr_registry);

    /* Should also call ins.init and adc.init */
    imu.init(IMU::COLD_START, delay, flash_leds, &adc_scheduler);
    imu.init_accel(delay, flash_leds);
}

void loop(void)
{
    Vector3f accel;
    Vector3f gyro;

    delay(1000);
    imu.update();
    accel = imu.get_accel();
    gyro = imu.get_gyro();

    Serial.printf("AX: %4.4f  AY: %4.4f  AZ: %4.4f  GX: %4.4f  GY: %4.4f  GZ: %4.4f\n",
                  accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
}
