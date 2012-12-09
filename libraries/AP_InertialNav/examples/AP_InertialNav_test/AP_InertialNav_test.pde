// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_InertialSensor MPU6000 driver.
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_GPS.h>             // ArduPilot GPS library
#include <I2C.h>                // Arduino I2C lib
#include <SPI.h>                // Arduino SPI lib
#include <SPI3.h>               // SPI3 library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_PeriodicProcess.h> // Parent header of Timer
#include <AP_TimerProcess.h>    // TimerProcess is the scheduler for MPU6000 reads.
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_InertialNav.h>
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;
AP_InertialSensor_MPU6000 ins;

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Doing INS startup...");

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHZ SPI rate

    isr_registry.init();
    scheduler.init(&isr_registry);

    // we need to stop the barometer from holding the SPI bus
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 delay, NULL, &scheduler);
}

void loop(void)
{
    Vector3f accel;
    Vector3f gyro;
    float temperature;

    delay(20);
    ins.update();
    gyro = ins.get_gyro();
    accel = ins.get_accel();
    temperature = ins.temperature();

    Serial.printf("AX: %f  AY: %f  AZ: %f  GX: %f  GY: %f  GZ: %f T=%f\n",
                  accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, temperature);
}
