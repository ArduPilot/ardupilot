/*
 *       Example of DataFlash library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <SPI.h>                        // Arduino SPI lib
#include <AP_Semaphore.h> // Required by DataFlash library
#include <DataFlash.h>

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console

#define HEAD_BYTE1 0xA3
#define HEAD_BYTE2 0x95

// NOTE:  You must uncomment one of the following two lines
DataFlash_APM2 DataFlash;                       // Uncomment this line if using APM2 hardware
//DataFlash_APM1	DataFlash;			// Uncomment this line if using APM1 hardware


void setup()
{
    Serial.begin(115200);
    DataFlash.Init();                            // DataFlash initialization

    Serial.println("Dataflash Log Test 1.0");

    // Test
    delay(20);
    DataFlash.ReadManufacturerID();
    delay(10);
    Serial.print("Manufacturer:");
    Serial.print(int(DataFlash.df_manufacturer));
    Serial.print(",");
    Serial.print(DataFlash.df_device);
    Serial.println();

    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    DataFlash.StartWrite(1);
    Serial.println("After testing perform erase before using DataFlash for logging!");
    Serial.println("");
    Serial.println("Writing to flash... wait...");
    for (int i = 0; i < 40; i++) {     // Write 1000 packets...
        // We write packets of binary data... (without worry about nothing more)
        DataFlash.WriteByte(HEAD_BYTE1);
        DataFlash.WriteByte(HEAD_BYTE2);
        DataFlash.WriteInt(2000 + i);
        DataFlash.WriteInt(2001 + i);
        DataFlash.WriteInt(2002 + i);
        DataFlash.WriteInt(2003 + i);
        DataFlash.WriteLong((long)i * 5000);
        DataFlash.WriteLong((long)i * 16268);
        DataFlash.WriteByte(0xA2);               // 2 bytes of checksum (example)
        DataFlash.WriteByte(0x4E);
        delay(10);
    }
    delay(100);
}

void loop()
{
    int i, tmp_int;
    byte tmp_byte1, tmp_byte2;
    long tmp_long;

    Serial.println("Start reading page 1...");

    DataFlash.StartRead(1);      // We start reading from page 1

    for (i = 0; i < 40; i++) {          // Read 200 packets...

        tmp_byte1 = DataFlash.ReadByte();
        tmp_byte2 = DataFlash.ReadByte();

        Serial.print("PACKET:");

        if ((tmp_byte1 == HEAD_BYTE1) && (tmp_byte1 == HEAD_BYTE1)) {
            // Read 4 ints...
            tmp_int = DataFlash.ReadInt();
            Serial.print(tmp_int);
            Serial.print(",");
            tmp_int = DataFlash.ReadInt();
            Serial.print(tmp_int);
            Serial.print(",");
            tmp_int = DataFlash.ReadInt();
            Serial.print(tmp_int);
            Serial.print(",");
            tmp_int = DataFlash.ReadInt();
            Serial.print(tmp_int);
            Serial.print(",");

            // Read 2 longs...
            tmp_long = DataFlash.ReadLong();
            Serial.print(tmp_long);
            Serial.print(",");
            tmp_long = DataFlash.ReadLong();
            Serial.print(tmp_long);
            Serial.print(";");

            // Read the checksum...
            tmp_byte1 = DataFlash.ReadByte();
            tmp_byte2 = DataFlash.ReadByte();
        }
        Serial.println();
    }
    Serial.println("");
    Serial.println("Test complete.  Test will repeat in 20 seconds");
    Serial.println("");
    delay(20000);
}
