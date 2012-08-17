
#include <stdint.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <I2C.h>
#include <SPI.h>
#include <Filter.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_Baro.h> // ArduPilot Mega ADC Library

FastSerialPort0(Serial);

AP_Baro_MS5611 baro;
Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

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
    SPI.setClockDivider(SPI_CLOCK_DIV32);     // 500khz for debugging, increase later

    baro.init(&scheduler);
    baro.calibrate(delay);
    timer = millis();
}

void loop()
{
    if((micros() - timer) > 100000UL) {
        timer = micros();
        baro.read();
        uint32_t read_time = micros() - timer;
        if (!baro.healthy) {
            Serial.println("not healthy");
            return;
        }
        Serial.print("Pressure:");
        Serial.print(baro.get_pressure());
        Serial.print(" Temperature:");
        Serial.print(baro.get_temperature());
        Serial.print(" Altitude:");
        Serial.print(baro.get_altitude());
        Serial.printf(" climb=%.2f t=%u samples=%u",
                      baro.get_climb_rate(),
                      (unsigned)read_time,
                      (unsigned)baro.get_pressure_samples());
        Serial.println();
    }
}
