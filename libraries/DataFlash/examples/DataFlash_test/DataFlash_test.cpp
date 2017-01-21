/*
 * Example of DataFlash library.
 * originally based on code by Jordi MuÒoz and Jose Julio
 */

#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define LOG_TEST_MSG 1

struct PACKED log_Test {
    LOG_PACKET_HEADER;
    uint16_t v1, v2, v3, v4;
    int32_t  l1, l2;
};

static const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_TEST_MSG, sizeof(log_Test),       
    "TEST", "HHHHii",        "V1,V2,V3,V4,L1,L2" }
};

#define NUM_PACKETS 500

static uint16_t log_num;

class DataFlashTest {
public:
    void setup();
    void loop();

private:

    DataFlash_Class dataflash{"DF Test 0.1"};
    void print_mode(AP_HAL::BetterStream *port, uint8_t mode);
};

static DataFlashTest dataflashtest;

void DataFlashTest::setup(void)
{
    dataflash.Init(log_structure, ARRAY_SIZE(log_structure));

    hal.console->printf("Dataflash Log Test 1.0\n");

    // Test
    hal.scheduler->delay(20);
    dataflash.ShowDeviceInfo(hal.console);

    if (dataflash.NeedPrep()) {
        hal.console->printf("Preparing dataflash...\n");
        dataflash.Prep();
    }

    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    dataflash.StartNewLog();
    log_num = dataflash.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);
    hal.console->printf("After testing perform erase before using DataFlash for logging!\n");
    hal.console->printf("\n");
    hal.console->printf("Writing to flash... wait...\n");

    uint32_t total_micros = 0;
    uint16_t i;

    for (i = 0; i < NUM_PACKETS; i++) {
        uint32_t start = AP_HAL::micros();
        // note that we use g++ style initialisers to make larger
        // structures easier to follow        
        struct log_Test pkt = {
            LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
            v1    : (uint16_t)(2000 + i),
            v2    : (uint16_t)(2001 + i),
            v3    : (uint16_t)(2002 + i),
            v4    : (uint16_t)(2003 + i),
            l1    : (int32_t)(i * 5000),
            l2    : (int32_t)(i * 16268)
        };
        dataflash.WriteBlock(&pkt, sizeof(pkt));
        total_micros += AP_HAL::micros() - start;
        hal.scheduler->delay(20);
    }

    hal.console->printf("Average write time %.1f usec/byte\n", 
                       (double)total_micros/((double)i*sizeof(struct log_Test)));

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    dataflash.flush();
#endif

    hal.scheduler->delay(100);
}

void DataFlashTest::loop(void)
{
    uint16_t start, end;

    hal.console->printf("Start read of log %u\n", log_num);

    dataflash.get_log_boundaries(log_num, start, end); 
    dataflash.LogReadProcess(log_num, start, end, 
                             FUNCTOR_BIND_MEMBER(&DataFlashTest::print_mode, void, AP_HAL::BetterStream *, uint8_t),//print_mode,
                             hal.console);
    hal.console->printf("\nTest complete.  Test will repeat in 20 seconds\n");
    hal.scheduler->delay(20000);
}

void DataFlashTest::print_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    port->printf("Mode(%u)", (unsigned)mode);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup()
{
    dataflashtest.setup();
}


void loop()
{
    dataflashtest.loop();
}

AP_HAL_MAIN();
