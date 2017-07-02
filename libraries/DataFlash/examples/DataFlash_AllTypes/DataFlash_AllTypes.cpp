/*
 * Write out two logs, each containing samples of each attribute type
 */

#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Format characters in the format string for binary log messages
struct PACKED log_TYP1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int8_t     b;
    uint8_t    B;
    int16_t    h;
    uint16_t   H;
    int32_t    i;
    uint32_t   I;
    float      f;
    double     d;
    char       n[4];
    char       N[16];
    char       Z[64];
};

struct PACKED log_TYP2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t   c;   // int16_t * 100
    uint16_t  C;   // uint16_t * 100
    int32_t   e;   // int32_t * 100
    uint32_t  E;   // uint32_t * 100
    int32_t   L;   // latitude/longitude
    uint8_t   M;   // flight mode
    int64_t   q;
    uint64_t  Q;
};


enum MyLogMessages {
    LOG_TYP1_MSG,
    LOG_TYP2_MSG,
};

static const struct LogStructure log_structure[] = {
    { LOG_TYP1_MSG, sizeof(log_TYP1),
      "TYP1", "QbBhHiIfdnNZ",        "TimeUS,b,B,h,H,i,I,f,d,n,N,Z" },
    { LOG_TYP2_MSG, sizeof(log_TYP2),
      "TYP2", "QcCeELMqQ",        "TimeUS,c,C,e,E,L,M,q,Q" },
    { LOG_MESSAGE_MSG, sizeof(log_Message),
      "MSG",  "QZ",     "TimeUS,Message"}
};

// these are identical to the entries in the above log-structure.  Not
// shared to maintain the visual similarity between the above
// structure and that in LogStructure.h
#define TYP1_FMT "QbBhHiIfdnNZ"
#define TYP1_LBL "TimeUS,b,B,h,H,i,I,f,d,n,N,Z"
#define TYP2_FMT "QcCeELMqQ"
#define TYP2_LBL "TimeUS,c,C,e,E,L,M,q,Q"

static uint16_t log_num;

class DataFlashTest_AllTypes : public AP_HAL::HAL::Callbacks {
public:
    void setup();
    void loop();

private:

    AP_Int32 unused;
    DataFlash_Class dataflash{"DF AllTypes 0.1", unused};
    void print_mode(AP_HAL::BetterStream *port, uint8_t mode);

    void Log_Write_TypeMessages();
    void Log_Write_TypeMessages_Log_Write();

    void flush_dataflash(DataFlash_Class &dataflash);
};

void DataFlashTest_AllTypes::flush_dataflash(DataFlash_Class &_dataflash)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    _dataflash.flush();
#else
    // flush is not available on e.g. px4 as it would be a somewhat
    // dangerous operation, but if we wait long enough (at time of
    // writing, 2 seconds, see DataFlash_File::_io_timer) the data
    // will go out.
    hal.scheduler->delay(3000);
#endif
}


void DataFlashTest_AllTypes::Log_Write_TypeMessages()
{
    dataflash.StartUnstartedLogging();
    log_num = dataflash.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);

    struct log_TYP1 typ1 = {
        LOG_PACKET_HEADER_INIT(LOG_TYP1_MSG),
        time_us : AP_HAL::micros64(),
        b : -17, // int8_t
        B : 42,  // uint8_t
        h : -12372,  // int16_t
        H : 19812,   // uint16_t
        i : -98234729,   // int32_t
        I : 74627293,    // uint32_t
        f : 35.87654,  // float
        d : 67.7393274658293,   // double
        n : { 'A', 'B', 'C', 'D' }, // char[4]
        // char[16]:
        N : { 'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P' },
        // char[64]:
        Z : { 'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
              'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
              'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
              'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P' }
    };
    dataflash.WriteBlock(&typ1, sizeof(typ1));

    struct log_TYP2 typ2 = {
        LOG_PACKET_HEADER_INIT(LOG_TYP2_MSG),
        time_us : AP_HAL::micros64(),
        c : -9823, // int16_t * 100
        C : 5436,  // uint16_t * 100
        e : -9209238,  // int32_t * 100
        E : 19239872,  // uint32_t * 100
        L : -3576543,   // uint32_t latitude/longitude;
        M : 5,          //   uint8_t;   // flight mode;
        q : -98239832498328,   // int64_t
        Q : 3432345232233432   // uint64_t
    };
    dataflash.WriteBlock(&typ2, sizeof(typ2));

    flush_dataflash(dataflash);

    dataflash.StopLogging();
}

void DataFlashTest_AllTypes::Log_Write_TypeMessages_Log_Write()
{
    dataflash.StartUnstartedLogging();
    log_num = dataflash.find_last_log();
    hal.console->printf("Using log number for Log_Write %u\n", log_num);

    dataflash.Log_Write("TYP3", TYP1_LBL, TYP1_FMT,
                        AP_HAL::micros64(),
                        -17, // int8_t
                        42,  // uint8_t
                        -12372,  // int16_t
                        19812,   // uint16_t
                        -98234729,   // int32_t
                        74627293,    // uint32_t
                        35.87654f,  // float
                        (double)67.7393274658293,   // double
                        "ABCD", // char[4]
                        // char[16]:
                        "ABCDEFGHIJKLMNOP",
                        // char[64]:
                        "ABCDEFGHIJKLMNOPABCDEFGHIJKLMNOPABCDEFGHIJKLMNOPABCDEFGHIJKLMNOP"
        );

    dataflash.Log_Write("TYP4", TYP2_LBL, TYP2_FMT,
                        AP_HAL::micros64(),
                        -9823, // int16_t * 100
                        5436,  // uint16_t * 100
                        -9209238,  // int32_t * 100
                        19239872,  // uint32_t * 100
                        -3576543,   // uint32_t latitude/longitude;
                        5,          //   uint8_t;   // flight mode;
                        -98239832498328,   // int64_t
                        3432345232233432   // uint64_t
        );

    flush_dataflash(dataflash);

    dataflash.StopLogging();
}

void DataFlashTest_AllTypes::setup(void)
{
    hal.console->printf("Dataflash All Types 1.0\n");

    dataflash.Init(log_structure, ARRAY_SIZE(log_structure));
    dataflash.set_vehicle_armed(true);

    // Test
    hal.scheduler->delay(20);
    dataflash.ShowDeviceInfo(hal.console);

    Log_Write_TypeMessages();
    Log_Write_TypeMessages_Log_Write();

    hal.console->printf("tests done\n");
}

void DataFlashTest_AllTypes::loop(void)
{
    hal.console->printf("all done\n");
    hal.scheduler->delay(1000);
}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;


static DataFlashTest_AllTypes dataflashtest;

AP_HAL_MAIN_CALLBACKS(&dataflashtest);
