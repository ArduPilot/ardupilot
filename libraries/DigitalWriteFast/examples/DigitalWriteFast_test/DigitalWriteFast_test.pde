// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *      Example of DigitalWriteFast library.
 *      DIYDrones.com
 *      DigitalFastWrite code by John Rains
 *      http://code.google.com/p/digitalwritefast
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <DigitalWriteFast.h> // Compass Library

#define LED_ON           LOW
#define LED_OFF          HIGH

#define APM_HARDWARE_APM1 1
#define APM_HARDWARE_APM2 2

// select your hardware
//#define CONFIG_APM_HARDWARE APM_HARDWARE_APM1
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM1
 # define A_LED_PIN        37
 # define B_LED_PIN        36
 # define C_LED_PIN        35
 # define LED_ON           HIGH
 # define LED_OFF          LOW
#elif CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
 # define A_LED_PIN        27
 # define B_LED_PIN        26
 # define C_LED_PIN        25
 # define LED_ON           LOW
 # define LED_OFF          HIGH
#endif

FastSerialPort0(Serial);

void flash_leds()
{
    uint8_t i;
    for(i=0; i<5; i++ ){
        digitalWriteFast(A_LED_PIN,LED_ON);
        digitalWriteFast(B_LED_PIN,LED_ON);
        digitalWriteFast(C_LED_PIN,LED_ON);
        delay(100);
        digitalWriteFast(A_LED_PIN,LED_OFF);
        digitalWriteFast(B_LED_PIN,LED_OFF);
        digitalWriteFast(C_LED_PIN,LED_OFF);
        delay(100);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("DigitalFastWrite library test version 1.0");

    // setup pins
    pinMode(A_LED_PIN, OUTPUT);
    pinMode(B_LED_PIN, OUTPUT);
    pinMode(C_LED_PIN, OUTPUT);
}

void loop()
{
    uint32_t start_time = micros();
    uint32_t regular_time, fast_time;
    
    // flash leds to show that we're operational
    flash_leds();

    // time regular digital write calls
    start_time = micros();
    for( uint16_t i=0; i<=500; i++ ) {
        digitalWrite(A_LED_PIN,LED_ON);
        digitalWrite(A_LED_PIN,LED_OFF);
    }
    regular_time = (micros() - start_time) / 1000;

    // flash leds to show that we're operational
    flash_leds();

    // time fast digital write calls
    start_time = micros();
    for( uint16_t i=0; i<=500; i++ ) {
        digitalWriteFast(A_LED_PIN,LED_ON);
        digitalWriteFast(A_LED_PIN,LED_OFF);
    }
    fast_time = (micros() - start_time) / 1000;

    // display results
    Serial.println("Timing results (in microseconds):");
    Serial.printf("DigitalWrite: %ul\n",regular_time);
    Serial.printf("DigitalWriteFast: %ul\n", fast_time);

    // turn everything off
    digitalWriteFast(A_LED_PIN,LED_OFF);
    digitalWriteFast(B_LED_PIN,LED_OFF);
    digitalWriteFast(C_LED_PIN,LED_OFF);

    // delay 10 seconds
    delay(10000);
}

