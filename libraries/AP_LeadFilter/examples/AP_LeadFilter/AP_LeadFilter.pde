/*
 *       Example of AP Lead_Filter library.
 *       Code by Jason Short. 2010
 *       DIYDrones.com
 *
 */

#include <FastSerial.h>
#include <AP_Common.h>          // ArduPilot Mega Common Library
#include <AP_Param.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_LeadFilter.h>      // GPS Lead filter

// set your hardware type here
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

Arduino_Mega_ISR_Registry isr_registry;

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console

AP_LeadFilter xLeadFilter;      // GPS lag filter

void setup()
{
    Serial.begin(115200);
    Serial.println("AP_LeadFilter test ver 1.0");

    delay(500);
}

void loop()
{
    int16_t velocity;
    int32_t position;
    int32_t new_position;
    int16_t i;

    position = 0;
    velocity = 0;
    xLeadFilter.clear();

    Serial.printf("------------------\n");
    Serial.printf("start position = 0, lag of 1sec.\n");
    for( i = 0; i < 10; i++ ) {
        // get updated position
        new_position = xLeadFilter.get_position(position, velocity);     // new position with velocity of 1 m/s
        Serial.printf("start pos: %ld, start vel: %d, end pos: %ld\n", (long int)position, (int)velocity, (long int)new_position);
        position = new_position;
        velocity += 100;
    }

    position = 0;
    velocity = 0;
    xLeadFilter.clear();

    Serial.printf("------------------\n");
    Serial.printf("start position = 0, lag of 200ms\n");
    for( i = 0; i < 10; i++ ) {
        // get updated position
        new_position = xLeadFilter.get_position(position, velocity, 0.200);     // new position with velocity of 1 m/s
        Serial.printf("start pos: %ld, start vel: %d, end pos: %ld\n", (long int)position, (int)velocity, (long int)new_position);
        position = new_position;
        velocity += 100;
    }

    delay(10000);
}




