#line 1 "./libraries/AP_HAL_VRBRAIN/examples/DataFlash_test_MP32/DataFlash_test_MP32.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       Example of DataFlash library.
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

// Libraries
#include <AP_HAL.h>
//#include <AP_HAL_AVR.h>
//#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

#include <AP_HAL_VRBRAIN.h>

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <DataFlash.h>



  void setup() ;
  static void callback(uint8_t msgid) ;
  void loop() ;
#line 24 "./libraries/AP_HAL_VRBRAIN/examples/DataFlash_test_MP32/DataFlash_test_MP32.pde"
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
DataFlash_APM2 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
DataFlash_APM1 DataFlash;
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
DataFlash_MP32  DataFlash;
#else
DataFlash_Empty DataFlash;
#endif


struct test_packet {
    LOG_PACKET_HEADER;
    uint16_t v1, v2, v3, v4;
    int32_t  l1, l2;
    uint8_t dummy[80];
};

#define NUM_PACKETS 20

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
    log_num = DataFlash.start_new_log();
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
        struct test_packet pkt = {
            LOG_PACKET_HEADER_INIT(7),
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
                        (double)total_micros/((float)i*sizeof(struct test_packet)));

    hal.scheduler->delay(100);
}

static void callback(uint8_t msgid)
{
    struct test_packet pkt;
    DataFlash.ReadPacket(&pkt, sizeof(pkt));
    hal.console->printf("PACKET: %02x,%u,%u,%u,%u,%ld,%ld\n",
                        (unsigned)msgid,
                        (unsigned)pkt.v1,
                        (unsigned)pkt.v2,
                        (unsigned)pkt.v3,
                        (unsigned)pkt.v4,
                        (long)pkt.l1,
                        (long)pkt.l2);
}

void loop()
{
    uint16_t start, end;

    hal.console->printf("Start read of log %u\n", log_num);

    DataFlash.get_log_boundaries(log_num, start, end); 
    DataFlash.log_read_process(log_num, start, end, callback);
    hal.console->printf("\nTest complete.  Test will repeat in 20 seconds\n");
    hal.scheduler->delay(20000);
}

AP_HAL_MAIN();
