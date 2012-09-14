/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <SPI3.h>                // Arduino SPI library

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot Mega SPI3 library test ver 0.1");

    delay(1000);

    // initialise SPI3 bus
    SPI3.begin();
    SPI3.setBitOrder(SPI3_MSBFIRST);
    SPI3.setDataMode(SPI3_MODE0);
    SPI3.setSpeed(SPI3_SPEED_2MHZ);

    delay(1000);
}

void loop()
{
    int value;

    // wait for user to enter something
    while( !Serial.available() ) {
        value = Serial.read();
        delay(20);
    }
}
