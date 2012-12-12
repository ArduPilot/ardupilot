/*
 *       Example of DataFlash library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

// Libraries
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <DataFlash.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>


#define HEAD_BYTE1 0xA3
#define HEAD_BYTE2 0x95

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
DataFlash_APM1 DataFlash;
#endif


void setup()
{
    DataFlash.Init();                            // DataFlash initialization

    hal.console->println("Dataflash Log Test 1.0");

    // Test
    hal.scheduler->delay(20);
    DataFlash.ReadManufacturerID();
    hal.scheduler->delay(10);
    hal.console->print("Manufacturer:");
    hal.console->print(int(DataFlash.df_manufacturer));
    hal.console->print(",");
    hal.console->print(DataFlash.df_device);
    hal.console->println();

    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    DataFlash.StartWrite(1);
    hal.console->println("After testing perform erase before using DataFlash for logging!");
    hal.console->println("");
    hal.console->println("Writing to flash... wait...");
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
        hal.scheduler->delay(10);
    }
    hal.scheduler->delay(100);
}

void loop()
{
    int i, tmp_int;
    uint8_t tmp_byte1, tmp_byte2;
    long tmp_long;

    hal.console->println("Start reading page 1...");

    DataFlash.StartRead(1);      // We start reading from page 1

    for (i = 0; i < 40; i++) {          // Read 200 packets...

        tmp_byte1 = DataFlash.ReadByte();
        tmp_byte2 = DataFlash.ReadByte();

        hal.console->print("PACKET:");

        if ((tmp_byte1 == HEAD_BYTE1) && (tmp_byte1 == HEAD_BYTE1)) {
            // Read 4 ints...
            tmp_int = DataFlash.ReadInt();
            hal.console->print(tmp_int);
            hal.console->print(",");
            tmp_int = DataFlash.ReadInt();
            hal.console->print(tmp_int);
            hal.console->print(",");
            tmp_int = DataFlash.ReadInt();
            hal.console->print(tmp_int);
            hal.console->print(",");
            tmp_int = DataFlash.ReadInt();
            hal.console->print(tmp_int);
            hal.console->print(",");

            // Read 2 longs...
            tmp_long = DataFlash.ReadLong();
            hal.console->print(tmp_long);
            hal.console->print(",");
            tmp_long = DataFlash.ReadLong();
            hal.console->print(tmp_long);
            hal.console->print(";");

            // Read the checksum...
            tmp_byte1 = DataFlash.ReadByte();
            tmp_byte2 = DataFlash.ReadByte();
        }
        hal.console->println();
    }
    hal.console->println("");
    hal.console->println("Test complete.  Test will repeat in 20 seconds");
    hal.console->println("");
    hal.scheduler->delay(20000);
}

AP_HAL_MAIN();
