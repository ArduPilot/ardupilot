/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       Example of DataFlash library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

// Libraries
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Compass.h>
#include <Filter.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>



const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#else
DataFlash_Empty DataFlash;
#endif

#define LOG_TEST_MSG 1

struct PACKED log_Test {
    LOG_PACKET_HEADER;
    uint16_t v1, v2, v3, v4;
    int32_t  l1, l2;
};

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_TEST_MSG, sizeof(log_Test),       
      "TEST", "HHHHii",        "V1,V2,V3,V4,L1,L2" }
};

#define NUM_PACKETS 500

static uint16_t log_num;

void setup()
{
    DataFlash.Init();                            // DataFlash initialization

    hal.console->println("Dataflash Log Test 1.0");

    // Test
    hal.scheduler->delay(20);
    DataFlash.ReadManufacturerID();
    hal.scheduler->delay(10);
    DataFlash.ShowDeviceInfo(hal.console);

    if (DataFlash.NeedErase()) {
        hal.console->println("Erasing...");
        DataFlash.EraseAll();
    }

    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    log_num = DataFlash.StartNewLog(sizeof(log_structure)/sizeof(log_structure[0]), log_structure);
    hal.console->printf("Using log number %u\n", log_num);
    hal.console->println("After testing perform erase before using DataFlash for logging!");
    hal.console->println("");
    hal.console->println("Writing to flash... wait...");

    uint32_t total_micros = 0;
    uint16_t i;

    for (i = 0; i < NUM_PACKETS; i++) {
        uint32_t start = hal.scheduler->micros();
        // note that we use g++ style initialisers to make larger
        // structures easier to follow        
        struct log_Test pkt = {
            LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
            v1    : 2000 + i,
            v2    : 2001 + i,
            v3    : 2002 + i,
            v4    : 2003 + i,
            l1    : (long)i * 5000,
            l2    : (long)i * 16268
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
        total_micros += hal.scheduler->micros() - start;
        hal.scheduler->delay(20);
    }

    hal.console->printf("Average write time %.1f usec/byte\n", 
                        (double)total_micros/((float)i*sizeof(struct log_Test)));

    hal.scheduler->delay(100);
}

static void
print_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
}

void loop()
{
    uint16_t start, end;

    hal.console->printf("Start read of log %u\n", log_num);

    DataFlash.get_log_boundaries(log_num, start, end); 
	DataFlash.LogReadProcess(log_num, start, end, 
                             sizeof(log_structure)/sizeof(log_structure[0]),
                             log_structure, 
                             print_mode,
                             hal.console);
    hal.console->printf("\nTest complete.  Test will repeat in 20 seconds\n");
    hal.scheduler->delay(20000);
}

AP_HAL_MAIN();
