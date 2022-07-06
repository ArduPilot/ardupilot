#pragma once

#include <AP_Logger/LogStructure.h>
#include "LogStructure_SBP.h"

#define LOG_IDS_FROM_GPS                        \
    LOG_GPS_MSG,                                \
    LOG_GPA_MSG,                                \
    LOG_GPS_RAW_MSG,                            \
    LOG_GPS_RAWH_MSG,                           \
    LOG_GPS_RAWS_MSG,                           \
    LOG_GPS_UBX1_MSG,                           \
    LOG_GPS_UBX2_MSG,                           \
    LOG_IDS_FROM_GPS_SBP


// @LoggerMessage: GPS
// @Description: Information received from GNSS systems attached to the autopilot
// @Field: TimeUS: Time since system startup
// @Field: I: GPS instance number
// @Field: Status: GPS Fix type; 2D fix, 3D fix etc.
// @Field: GMS: milliseconds since start of GPS Week
// @Field: GWk: weeks since 5 Jan 1980
// @Field: NSats: number of satellites visible
// @Field: HDop: horizontal precision
// @Field: Lat: latitude
// @Field: Lng: longitude
// @Field: Alt: altitude
// @Field: Spd: ground speed
// @Field: GCrs: ground course
// @Field: VZ: vertical speed
// @Field: Yaw: vehicle yaw
// @Field: U: boolean value indicating whether this GPS is in use
struct PACKED log_GPS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint8_t  status;
    uint32_t gps_week_ms;
    uint16_t gps_week;
    uint8_t  num_sats;
    uint16_t hdop;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    float    ground_speed;
    float    ground_course;
    float    vel_z;
    float    yaw;
    uint8_t  used;
};

// @LoggerMessage: GPA
// @Description: GPS accuracy information
// @Field: I: GPS instance number
// @Field: TimeUS: Time since system startup
// @Field: VDop: vertical degree of procession
// @Field: HAcc: horizontal position accuracy
// @Field: VAcc: vertical position accuracy
// @Field: SAcc: speed accuracy
// @Field: YAcc: yaw accuracy
// @Field: VV: true if vertical velocity is available
// @Field: SMS: time since system startup this sample was taken
// @Field: Delta: system time delta between the last two reported positions
struct PACKED log_GPA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t vdop;
    uint16_t hacc;
    uint16_t vacc;
    uint16_t sacc;
    float    yaw_accuracy;
    uint8_t  have_vv;
    uint32_t sample_ms;
    uint16_t delta_ms;
};

/*
  UBlox logging
 */

// @LoggerMessage: UBX1
// @Description: uBlox-specific GPS information (part 1)
// @Field: TimeUS: Time since system startup
// @Field: Instance: GPS instance number
// @Field: noisePerMS: noise level as measured by GPS
// @Field: jamInd: jamming indicator; higher is more likely jammed
// @Field: aPower: antenna power indicator; 2 is don't know
// @Field: agcCnt: automatic gain control monitor
// @Field: config: bitmask for messages which haven't been seen
struct PACKED log_Ubx1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t noisePerMS;
    uint8_t  jamInd;
    uint8_t  aPower;
    uint16_t agcCnt;
    uint32_t config;
};

// @LoggerMessage: UBX2
// @Description: uBlox-specific GPS information (part 2)
// @Field: TimeUS: Time since system startup
// @Field: Instance: GPS instance number
// @Field: ofsI: imbalance of I part of complex signal
// @Field: magI: magnitude of I part of complex signal
// @Field: ofsQ: imbalance of Q part of complex signal
// @Field: magQ: magnitude of Q part of complex signal
struct PACKED log_Ubx2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
};

// @LoggerMessage: GRAW
// @Description: Raw uBlox data
// @Field: TimeUS: Time since system startup
// @Field: WkMS: receiver TimeOfWeek measurement
// @Field: Week: GPS week
// @Field: numSV: number of space vehicles seen
// @Field: sv: space vehicle number of first vehicle
// @Field: cpMes: carrier phase measurement
// @Field: prMes: pseudorange measurement
// @Field: doMes: Doppler measurement
// @Field: mesQI: measurement quality index
// @Field: cno: carrier-to-noise density ratio
// @Field: lli: loss of lock indicator
struct PACKED log_GPS_RAW {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t iTOW;
    int16_t week;
    uint8_t numSV;
    uint8_t sv;
    double cpMes;
    double prMes;
    float doMes;
    int8_t mesQI;
    int8_t cno;
    uint8_t lli;
};

// @LoggerMessage: GRXH
// @Description: Raw uBlox data - header
// @Field: TimeUS: Time since system startup
// @Field: rcvTime: receiver TimeOfWeek measurement
// @Field: week: GPS week
// @Field: leapS: GPS leap seconds
// @Field: numMeas: number of space-vehicle measurements to follow
// @Field: recStat: receiver tracking status bitfield
struct PACKED log_GPS_RAWH {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double rcvTow;
    uint16_t week;
    int8_t leapS;
    uint8_t numMeas;
    uint8_t recStat;
};

// @LoggerMessage: GRXS
// @Description: Raw uBlox data - space-vehicle data
// @Field: TimeUS: Time since system startup
// @Field: prMes: Pseudorange measurement
// @Field: cpMes: Carrier phase measurement
// @Field: doMes: Doppler measurement
// @Field: gnss: GNSS identifier
// @Field: sv: Satellite identifier
// @Field: freq: GLONASS frequency slot
// @Field: lock: carrier phase locktime counter
// @Field: cno: carrier-to-noise density ratio
// @Field: prD: estimated pseudorange measurement standard deviation
// @Field: cpD: estimated carrier phase measurement standard deviation
// @Field: doD: estimated Doppler measurement standard deviation
// @Field: trk: tracking status bitfield
struct PACKED log_GPS_RAWS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    double prMes;
    double cpMes;
    float doMes;
    uint8_t gnssId;
    uint8_t svId;
    uint8_t freqId;
    uint16_t locktime;
    uint8_t cno;
    uint8_t prStdev;
    uint8_t cpStdev;
    uint8_t doStdev;
    uint8_t trkStat;
};

#define LOG_STRUCTURE_FROM_GPS \
    { LOG_GPS_MSG, sizeof(log_GPS), \
      "GPS",  "QBBIHBcLLeffffB", "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U", "s#---SmDUmnhnh-", "F----0BGGB000--" , true }, \
    { LOG_GPA_MSG,  sizeof(log_GPA), \
      "GPA",  "QBCCCCfBIH", "TimeUS,I,VDop,HAcc,VAcc,SAcc,YAcc,VV,SMS,Delta", "s#mmmnd-ss", "F-BBBB0-CC" , true }, \
    { LOG_GPS_UBX1_MSG, sizeof(log_Ubx1), \
      "UBX1", "QBHBBHI",  "TimeUS,Instance,noisePerMS,jamInd,aPower,agcCnt,config", "s#-----", "F------"  , true }, \
    { LOG_GPS_UBX2_MSG, sizeof(log_Ubx2), \
      "UBX2", "QBbBbB", "TimeUS,Instance,ofsI,magI,ofsQ,magQ", "s#----", "F-----" , true }, \
    { LOG_GPS_RAW_MSG, sizeof(log_GPS_RAW), \
      "GRAW", "QIHBBddfBbB", "TimeUS,WkMS,Week,numSV,sv,cpMes,prMes,doMes,mesQI,cno,lli", "s--S-------", "F--0-------" , true }, \
    { LOG_GPS_RAWH_MSG, sizeof(log_GPS_RAWH), \
      "GRXH", "QdHbBB", "TimeUS,rcvTime,week,leapS,numMeas,recStat", "s-----", "F-----" , true }, \
    { LOG_GPS_RAWS_MSG, sizeof(log_GPS_RAWS), \
      "GRXS", "QddfBBBHBBBBB", "TimeUS,prMes,cpMes,doMes,gnss,sv,freq,lock,cno,prD,cpD,doD,trk", "s------------", "F------------" , true }, \
    LOG_STRUCTURE_FROM_GPS_SBP
