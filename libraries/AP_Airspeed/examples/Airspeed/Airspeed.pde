/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *   Airspeed.pde - airspeed example sketch
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License
 *   as published by the Free Software Foundation; either version 2.1
 *   of the License, or (at your option) any later version.
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_AnalogSource.h>
#include <AP_AnalogSource_Arduino.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <Filter.h>
#include <AP_Buffer.h>
#include <AP_Airspeed.h>

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

FastSerialPort0(Serial);

AP_AnalogSource_Arduino pin0(0);
AP_Airspeed airspeed(&pin0);

void setup()
{
    Serial.begin(115200, 128, 128);
    Serial.println("ArduPilot Airspeed library test");

    isr_registry.init();
    scheduler.init(&isr_registry);
    AP_AnalogSource_Arduino::init_timer(&scheduler);

    pinMode(0, INPUT);

    airspeed.calibrate(delay);
}

void loop(void)
{
    static uint32_t timer;
    if((millis() - timer) > 100) {
        timer = millis();
        airspeed.read();
        Serial.printf("airspeed %.2f\n", airspeed.get_airspeed());
    }
}
