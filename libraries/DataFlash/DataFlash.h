/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_AHRS.h>
#include <stdint.h>

class DataFlash_Class
{
public:
    // initialisation
    virtual void Init(void) = 0;
    virtual bool CardInserted(void) = 0;

    // erase handling
    virtual bool NeedErase(void) = 0;
    virtual void EraseAll() = 0;

    /* Write a block of data at current offset */
    virtual void WriteBlock(const void *pBuffer, uint16_t size) = 0;

    // high level interface
    virtual uint16_t find_last_log(void) = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual uint16_t get_num_logs(void) = 0;
    virtual void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                uint8_t num_types,
                                const struct LogStructure *structure,
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;

    /* logging methods common to all vehicles */
    uint16_t StartNewLog(uint8_t num_types,
                         const struct LogStructure *structure);
    void Log_Write_Format(const struct LogStructure *structure);
    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const GPS *gps, int32_t relative_alt);
    void Log_Write_IMU(const AP_InertialSensor *ins);
    void Log_Write_Message(const char *message);
    void Log_Write_Message_P(const prog_char_t *message);

	/*
      every logged packet starts with 3 bytes
    */
    struct log_Header {
        uint8_t head1, head2, msgid;
    };

protected:
    /*
    read and print a log entry using the format strings from the given structure
    */
    void _print_log_entry(uint8_t msg_type, 
                          uint8_t num_types, 
                          const struct LogStructure *structure,
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);
    
    void Log_Write_Parameter(const AP_Param *ap, const AP_Param::ParamToken &token, 
                             enum ap_var_type type);
    void Log_Write_Parameters(void);
    virtual uint16_t start_new_log(void) = 0;

    /*
      read a block
    */
    virtual void ReadBlock(void *pkt, uint16_t size) = 0;

};

/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  L   : int32_t latitude/longitude
  M   : uint8_t flight mode
 */

// structure used to define logging format
struct LogStructure {
    uint8_t msg_type;
    uint8_t msg_len;
    const char name[5];
    const char format[16];
    const char labels[64];
};

/*
  log structures common to all vehicle types
 */
struct PACKED log_Format {
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_Parameter {
    LOG_PACKET_HEADER;
    char name[16];
    float value;
};

struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint8_t  status;
    uint32_t gps_time;
    uint8_t  num_sats;
    int16_t  hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    char msg[64];
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
};

#define LOG_COMMON_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "Nf",        "Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  "BIBcLLeeEe", "Status,Time,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs" }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  "ffffff",     "GyrX,GyrY,GyrZ,AccX,AccY,AccZ" }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "Z",     "Message" }

// message types for common messages
#define LOG_FORMAT_MSG	  128
#define LOG_PARAMETER_MSG 129
#define LOG_GPS_MSG		  130
#define LOG_IMU_MSG		  131
#define LOG_MESSAGE_MSG	  132

#include "DataFlash_Block.h"
#include "DataFlash_File.h"

#endif
