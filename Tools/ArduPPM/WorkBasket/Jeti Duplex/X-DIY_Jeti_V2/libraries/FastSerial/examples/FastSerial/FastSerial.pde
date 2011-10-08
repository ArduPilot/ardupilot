// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the FastSerial driver.
//
// This code is placed into the public domain.
//

//
// Include the FastSerial library header.
//
// Note that this causes the standard Arduino Serial* driver to be
// disabled.
//
#include <FastSerial.h>

//
// Create a FastSerial driver that looks just like the stock Arduino
// driver.
//
FastSerialPort0(Serial);

//
// To create a driver for a different serial port, on a board that
// supports more than one, use the appropriate macro:
//
//FastSerialPort2(Serial2);


void setup(void)
{
        //
        // Set the speed for our replacement serial port.
        //
	Serial.begin(38400);

        //
        // And send a message.
        //
        Serial.println("begin");
}

void
loop(void)
{
    int    c;

    //
    // Perform a simple loopback operation.
    //
    c = Serial.read();
    if (-1 != c)
        Serial.write(c);
}

