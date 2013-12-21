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
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <stdint.h>

class DataFlash_Class
{
public:
    // initialisation
    virtual void Init(const struct LogStructure *structure, uint8_t num_types);
    virtual bool CardInserted(void) = 0;

    // erase handling
    virtual bool NeedErase(void) = 0;
    virtual void EraseAll() = 0;

    /* Write a block of data at current offset */
    virtual void WriteBlock(const void *pBuffer, uint16_t size) = 0;

    // high level interface
    virtual uint16_t find_last_log(void) = 0;
    virtual void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) = 0;
    virtual void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) = 0;
    virtual int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) = 0;
    virtual uint16_t get_num_logs(void) = 0;
    virtual void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page, 
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port) = 0;
    virtual void DumpPageInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ShowDeviceInfo(AP_HAL::BetterStream *port) = 0;
    virtual void ListAvailableLogs(AP_HAL::BetterStream *port) = 0;

    /* logging methods common to all vehicles */
    uint16_t StartNewLog(void);
    void EnableWrites(bool enable) { _writes_enabled = enable; }
    void Log_Write_Format(const struct LogStructure *structure);
    void Log_Write_Parameter(const char *name, float value);
    void Log_Write_GPS(const GPS *gps, int32_t relative_alt);
    void Log_Write_GPS2(const GPS *gps);
    void Log_Write_IMU(const AP_InertialSensor &ins);
    void Log_Write_RCIN(void);
    void Log_Write_RCOUT(void);
    void Log_Write_Baro(AP_Baro &baro);
    void Log_Write_Power(void);
    void Log_Write_AHRS2(AP_AHRS &ahrs);
#if AP_AHRS_NAVEKF_AVAILABLE
    void Log_Write_EKF(AP_AHRS_NavEKF &ahrs);
#endif
    void Log_Write_Message(const char *message);
    void Log_Write_Message_P(const prog_char_t *message);

    bool logging_started(void) const { return log_write_started; }

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
                          void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                          AP_HAL::BetterStream *port);
    
    void Log_Fill_Format(const struct LogStructure *structure, struct log_Format &pkt);
    void Log_Write_Parameter(const AP_Param *ap, const AP_Param::ParamToken &token, 
                             enum ap_var_type type);
    void Log_Write_Parameters(void);
    virtual uint16_t start_new_log(void) = 0;

    const struct LogStructure *_structures;
    uint8_t _num_types;
    bool _writes_enabled;
    bool log_write_started;

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
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    int16_t  hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  rel_altitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
    float    vel_z;
    uint32_t apm_time;
};

struct PACKED log_GPS2 {
    LOG_PACKET_HEADER;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    int16_t  hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    uint32_t ground_speed;
    int32_t  ground_course;
    float    vel_z;
    uint32_t apm_time;
    uint8_t  dgps_numch;
    uint32_t dgps_age;
};

struct PACKED log_Message {
    LOG_PACKET_HEADER;
    char msg[64];
};

struct PACKED log_IMU {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
};

struct PACKED log_RCIN {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
};

struct PACKED log_RCOUT {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
    uint16_t chan5;
    uint16_t chan6;
    uint16_t chan7;
    uint16_t chan8;
};

struct PACKED log_BARO {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float   altitude;
    float   pressure;
    int16_t temperature;
};

struct PACKED log_AHRS {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float alt;
    int32_t lat;
    int32_t lng;
};

struct PACKED log_POWR {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t Vcc;
    uint16_t Vservo;
    uint16_t flags;
};

struct PACKED log_EKF1 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    float velN;
    float velE;
    float velD;
    float posN;
    float posE;
    float posD;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
};

struct PACKED log_EKF2 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int8_t accX;
    int8_t accY;
    int8_t accZ;
    int16_t windN;
    int16_t windE;
    int16_t magN;
    int16_t magE;
    int16_t magD;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
};

struct PACKED log_EKF3 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t innovVN;
    int16_t innovVE;
    int16_t innovVD;
    int16_t innovPN;
    int16_t innovPE;
    int16_t innovPD;
    int16_t innovMX;
    int16_t innovMY;
    int16_t innovMZ;
    int16_t innovVT;
};

struct PACKED log_EKF4 {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t sqrtvarVN;
    int16_t sqrtvarVE;
	int16_t sqrtvarVD;
    int16_t sqrtvarPN;
    int16_t sqrtvarPE;
    int16_t sqrtvarPD;
    int16_t sqrtvarMX;
    int16_t sqrtvarMY;
    int16_t sqrtvarMZ;
    int16_t sqrtvarVT;
};

#define LOG_COMMON_STRUCTURES \
    { LOG_FORMAT_MSG, sizeof(log_Format), \
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },    \
    { LOG_PARAMETER_MSG, sizeof(log_Parameter), \
      "PARM", "Nf",        "Name,Value" },    \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  "BIHBcLLeeEefI", "Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,T" }, \
    { LOG_GPS2_MSG, sizeof(log_GPS2), \
      "GPS2",  "BIHBcLLeEefIBI", "Status,TimeMS,Week,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,T,DSc,DAg" }, \
    { LOG_IMU_MSG, sizeof(log_IMU), \
      "IMU",  "Iffffff",     "TimeMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ" }, \
    { LOG_IMU2_MSG, sizeof(log_IMU), \
      "IMU2",  "Iffffff",     "TimeMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ" }, \
    { LOG_MESSAGE_MSG, sizeof(log_Message), \
      "MSG",  "Z",     "Message"}, \
    { LOG_RCIN_MSG, sizeof(log_RCIN), \
      "RCIN",  "Ihhhhhhhh",     "TimeMS,Chan1,Chan2,Chan3,Chan4,Chan5,Chan6,Chan7,Chan8" }, \
    { LOG_RCOUT_MSG, sizeof(log_RCOUT), \
      "RCOU",  "Ihhhhhhhh",     "TimeMS,Chan1,Chan2,Chan3,Chan4,Chan5,Chan6,Chan7,Chan8" }, \
    { LOG_BARO_MSG, sizeof(log_BARO), \
      "BARO",  "Iffc",     "TimeMS,Alt,Press,Temp" }, \
    { LOG_POWR_MSG, sizeof(log_POWR), \
      "POWR","ICCH","TimeMS,Vcc,VServo,Flags" },  \
    { LOG_AHR2_MSG, sizeof(log_AHRS), \
      "AHR2","IccCfLL","TimeMS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_SIMSTATE_MSG, sizeof(log_AHRS), \
      "SIM","IccCfLL","TimeMS,Roll,Pitch,Yaw,Alt,Lat,Lng" }, \
    { LOG_EKF1_MSG, sizeof(log_EKF1), \
      "EKF1","IccCffffffccc","TimeMS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,GX,GY,GZ" }, \
    { LOG_EKF2_MSG, sizeof(log_EKF2), \
      "EKF2","Ibbbcchhhhhh","TimeMS,AX,AY,AZ,VWN,VWE,MN,ME,MD,MX,MY,MZ" }, \
    { LOG_EKF3_MSG, sizeof(log_EKF3), \
      "EKF3","Icccccchhhc","TimeMS,IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IVT" }, \
    { LOG_EKF4_MSG, sizeof(log_EKF4), \
      "EKF4","Icccccchhhc","TimeMS,SVN,SVE,SVD,SPN,SPE,SPD,SMX,SMY,SMZ,SVT" }

// message types for common messages
#define LOG_FORMAT_MSG	  128
#define LOG_PARAMETER_MSG 129
#define LOG_GPS_MSG		  130
#define LOG_IMU_MSG		  131
#define LOG_MESSAGE_MSG	  132
#define LOG_RCIN_MSG      133
#define LOG_RCOUT_MSG     134
#define LOG_IMU2_MSG	  135
#define LOG_BARO_MSG	  136
#define LOG_POWR_MSG	  137
#define LOG_AHR2_MSG	  138
#define LOG_SIMSTATE_MSG  139
#define LOG_EKF1_MSG      140
#define LOG_EKF2_MSG      141
#define LOG_EKF3_MSG      142
#define LOG_EKF4_MSG      143
#define LOG_GPS2_MSG	  144

#include "DataFlash_Block.h"
#include "DataFlash_File.h"

#endif
