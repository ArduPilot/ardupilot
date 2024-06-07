/*
 * Write out two logs, each containing samples of each attribute type
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Format characters in the format string for binary log messages
struct PACKED log_TYP1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t    a[32];
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
static_assert(sizeof(log_TYP1) < 256, "log_TYP1 is oversize");

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
    { LOG_FORMAT_MSG,
      sizeof(log_Format),
      "FMT",
      "BBnNZ",
      "Type,Length,Name,Format,Columns",
      "-b---",
      "-----" },
    { LOG_UNIT_MSG, sizeof(log_Unit),
      "UNIT", "QbZ",      "TimeUS,Id,Label", "s--","F--" },
    { LOG_FORMAT_UNITS_MSG, sizeof(log_Format_Units),
      "FMTU", "QBNN",      "TimeUS,FmtType,UnitIds,MultIds","s---", "F---" },
    { LOG_MULT_MSG, sizeof(log_Format_Multiplier),
      "MULT", "Qbd",      "TimeUS,Id,Mult", "s--","F--" },

    { LOG_TYP1_MSG,
      sizeof(log_TYP1),
      "TYP1",
      "QabBhHiIfdnNZ",
      "TimeUS,a,b,B,h,H,i,I,f,d,n,N,Z",
      "s------------",
      "F------------"
    },
    { LOG_TYP2_MSG,
      sizeof(log_TYP2),
      "TYP2",
      "QcCeELMqQ",
      "TimeUS,c,C,e,E,L,M,q,Q",
      "s--------",
      "F--------"
    },
    { LOG_MESSAGE_MSG,
      sizeof(log_Message),
      "MSG",
      "QZ",
      "TimeUS,Message",
      "s-",
      "F-"}
};

// these are identical to the entries in the above log-structure.  Not
// shared to maintain the visual similarity between the above
// structure and that in LogStructure.h
#define TYP1_FMT "QabBhHiIfdnNZ"
#define TYP1_LBL "TimeUS,b,B,h,H,i,I,f,d,n,N,Z"
#define TYP2_FMT "QcCeELMqQ"
#define TYP2_LBL "TimeUS,c,C,e,E,L,M,q,Q"

static uint16_t log_num;

class AP_LoggerTest_AllTypes : public AP_HAL::HAL::Callbacks {
public:
    void setup() override;
    void loop() override;

private:

    AP_Int32 log_bitmask;
    AP_Logger logger;
    AP_Scheduler scheduler;

    void Log_Write_TypeMessages();
    void Log_Write_TypeMessages_Log_Write();

    void flush_logger(AP_Logger &logger);
};

void AP_LoggerTest_AllTypes::flush_logger(AP_Logger &_logger)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    _logger.flush();
#else
    // flush is not available on e.g. stm32 as it would be a somewhat
    // dangerous operation, but if we wait long enough (at time of
    // writing, 2 seconds, see AP_Logger_File::_io_timer) the data
    // will go out.
    hal.scheduler->delay(3000);
#endif
}


void AP_LoggerTest_AllTypes::Log_Write_TypeMessages()
{
    log_num = logger.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);

    hal.console->printf("Writing out a few messages to get formats out...");
    logger.Write_Message("Start 1");

    struct log_TYP1 typ1{
        LOG_PACKET_HEADER_INIT(LOG_TYP1_MSG),
        time_us : AP_HAL::micros64(),
        a : { -32768, 32767, 1, -1, 0, 19 }, // int16[32]
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
    logger.WriteBlock(&typ1, sizeof(typ1));

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
    logger.WriteBlock(&typ2, sizeof(typ2));

    flush_logger(logger);

    logger.StopLogging();
}

void AP_LoggerTest_AllTypes::Log_Write_TypeMessages_Log_Write()
{
    log_num = logger.find_last_log();
    hal.console->printf("Using log number for Log_Write %u\n", log_num);

    hal.console->printf("Writing out a few messages to get formats out...");
    logger.Write_Message("Start 2");

    logger.Write("TYPn",
                 "TimeUS,Str",
                 "Qn",
                 AP_HAL::micros64(),
                 "ABCD");

    const int16_t a[32] = { -32768, 32767, 1, -1, 0, 17 };

    logger.Write("TYPa",
                 "TimeUS,Arr",
                 "Qa",
                 AP_HAL::micros64(),
                 a);

    logger.Write("TYP3",
                 TYP1_LBL,
                 TYP1_FMT,
                 AP_HAL::micros64(),
                 a, // int16[32]
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

    logger.Write("TYP4",
                 TYP2_LBL,
                 TYP2_FMT,
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

    // emit a message which contains NaNs:
    logger.Write("NANS", "f,d,bf,bd", "fdfd",  logger.quiet_nanf(), logger.quiet_nan(), NAN, NAN);

    flush_logger(logger);

    logger.StopLogging();
}

void AP_LoggerTest_AllTypes::setup(void)
{
    hal.console->printf("Logger All Types 1.0\n");

    log_bitmask.set((uint32_t)-1);
    logger.init(log_bitmask, log_structure, ARRAY_SIZE(log_structure));
    logger.set_vehicle_armed(true);
    logger.Write_Message("AP_Logger Test");

    // Test
    hal.scheduler->delay(20);

    Log_Write_TypeMessages();
    Log_Write_TypeMessages_Log_Write();

    hal.console->printf("tests done\n");
}

void AP_LoggerTest_AllTypes::loop(void)
{
    hal.console->printf("all done\n");
    hal.scheduler->delay(1000);
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;


static AP_LoggerTest_AllTypes loggertest;

AP_HAL_MAIN_CALLBACKS(&loggertest);
