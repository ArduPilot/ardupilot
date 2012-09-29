// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_TimerProcess library
//

#include <FastSerial.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_Math.h>
#include <AP_Common.h>

FastSerialPort0(Serial);
FastSerialPort1(Serial1);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess scheduler;

uint32_t counter;

void ping(uint32_t now)
{
    counter++;
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("AP_TimerProcess Test ver 0.1");

    isr_registry.init();
    scheduler.init(&isr_registry);

    // register our ping function
    scheduler.register_process(ping);
}

void loop(void)
{
    static uint32_t secs = 0;

    // check if 1second has passed
    if( counter >= 1000 ) {
        counter -= 1000;
        secs++;
        Serial.print("Seconds: ");
        Serial.println(secs);
    }
}
