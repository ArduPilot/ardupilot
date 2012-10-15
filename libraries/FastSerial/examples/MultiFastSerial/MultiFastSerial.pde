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
#include <AP_Common.h>
/* Not required by this sketch, but required by AP_Common */
#include <AP_Math.h>

//
// Create a FastSerial driver that looks just like the stock Arduino
// driver, on each serial port.
//
FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);


void setup(void)
{
        //
        // Set the speed for our replacement serial port.
        //
        Serial.begin(115200);
        Serial1.begin(115200);
        Serial2.begin(115200);
        Serial3.begin(115200);

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
        
        Serial1.println("hello serial1");
        Serial2.println("hello serial2");
        Serial3.println("hello serial3");

}

void
loop(void)
{
    int    c;

    //
    // Perform a simple loopback operation on each port.
    //
    c = Serial.read();
    if (-1 != c)
        Serial.write(c);

    c = Serial1.read();
    if (-1 != c)
        Serial1.write(c);

    c = Serial2.read();
    if (-1 != c)
        Serial2.write(c);
    
    c = Serial3.read();
    if (-1 != c)
        Serial3.write(c);
}
