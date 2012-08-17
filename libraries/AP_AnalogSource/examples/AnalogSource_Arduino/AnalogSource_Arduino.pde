// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
 *       Example of AnalogSource Arduino
 */

#include <FastSerial.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_AnalogSource.h>

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

FastSerialPort0(Serial);

AP_AnalogSource_Arduino vcc_source(ANALOG_PIN_VCC);
AP_AnalogSource_Arduino pin0(0);
AP_AnalogSource_Arduino pin1(1);


void setup()
{
    Serial.begin(115200);
    isr_registry.init();
    scheduler.init(&isr_registry);
    AP_AnalogSource_Arduino::init_timer(&scheduler);
}

void loop()
{
    while (true) {
        delay(1000);
        Serial.printf("Vcc: %u PIN0: %u  PIN1: %u  PIN1_avg: %u\n",
                      (unsigned)vcc_source.read_vcc(),
                      (unsigned)pin0.read_raw(),
                      (unsigned)pin1.read_raw(),
                      (unsigned)pin1.read_average());
    }
}
