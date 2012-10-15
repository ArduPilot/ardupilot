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

        do {
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 1000);
        
        do {
          Serial1.println("hello serial1");
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 2000);

        do {
          Serial2.println("hello serial2");
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 3000);

        do {
          Serial3.println("hello serial3");
          Serial.print("hello serial0 millis: ");
          Serial.println(millis(), DEC);
        } while (millis() < 4000);

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
