/*
 *       Example of APM_BMP085 (absolute pressure sensor) library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <FastSerial.h>
#include <I2C.h>
#include <SPI.h>
#include <Filter.h>
#include <AP_Baro.h> // ArduPilot Mega BMP085 Library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AverageFilter.h>
#include <AP_Buffer.h>

#ifndef APM2_HARDWARE
 # define APM2_HARDWARE 0
#endif

AP_Baro_BMP085 APM_BMP085(APM2_HARDWARE);

unsigned long timer;

FastSerialPort0(Serial);


Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot Mega BMP085 library test");
    Serial.println("Initialising barometer..."); delay(100);

    I2c.begin();
    I2c.timeOut(20);

    //I2c.setSpeed(true);

    isr_registry.init();
    scheduler.init(&isr_registry);

    if (!APM_BMP085.init(&scheduler)) {
        Serial.println("Barometer initialisation FAILED\n");
    }
    Serial.println("initialisation complete."); delay(100);
    delay(1000);
    timer = micros();
}

void loop()
{
    float tmp_float;
    float Altitude;

    if((micros()- timer) > 50000L) {
        timer = micros();
        APM_BMP085.read();
        unsigned long read_time = micros() - timer;
        if (!APM_BMP085.healthy) {
            Serial.println("not healthy");
            return;
        }
        Serial.print("Pressure:");
        Serial.print(APM_BMP085.get_pressure());
        Serial.print(" Temperature:");
        Serial.print(APM_BMP085.get_temperature());
        Serial.print(" Altitude:");
        tmp_float = (APM_BMP085.get_pressure() / 101325.0);
        tmp_float = pow(tmp_float, 0.190295);
        Altitude = 44330.0 * (1.0 - tmp_float);
        Serial.print(Altitude);
        Serial.printf(" t=%u", (unsigned)read_time);
        Serial.println();
    }
}
