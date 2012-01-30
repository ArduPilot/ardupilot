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

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

# undef PSTR
# define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); \
                (prog_char_t *)&__c[0];}))

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
	Serial.begin(115200);

        //
        // Test printing things
        //
        Serial.print("test");
        Serial.println(" begin");
        Serial.println(1000);
        Serial.println(1000, 8);
        Serial.println(1000, 10);
        Serial.println(1000, 16);
        Serial.println_P(PSTR("progmem"));
        Serial.printf("printf %d %u %#x %p %f %S\n", -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
        Serial.printf_P(PSTR("printf_P %d %u %#x %p %f %S\n"), -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
        Serial.println("done");
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

