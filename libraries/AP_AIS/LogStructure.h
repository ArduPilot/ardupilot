#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_AIS_config.h"

#define LOG_IDS_FROM_AIS \
    LOG_AIS_RAW_MSG,\
    LOG_AIS_MSG1, \
    LOG_AIS_MSG4, \
    LOG_AIS_MSG5

struct PACKED log_AIS_raw {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t num;
    uint8_t total;
    uint8_t ID;
    char payload[64];
};

struct PACKED log_AIS_msg1 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    uint8_t repeat;
    uint32_t mmsi;
    uint8_t nav;
    int8_t rot;
    uint16_t sog;
    int8_t pos_acc;
    int32_t lon;
    int32_t lat;
    uint16_t cog;
    uint16_t head;
    uint8_t sec_utc;
    uint8_t maneuver;
    uint8_t raim;
    uint32_t radio;
};

struct PACKED log_AIS_msg4 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t repeat;
    uint32_t mmsi;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t fix;
    int32_t lon;
    int32_t lat;
    uint8_t epfd;
    uint8_t raim;
    uint32_t radio;
};

struct PACKED log_AIS_msg5 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t repeat;
    uint32_t mmsi;
    uint8_t ver;
    uint32_t imo;
    char call_sign[16];
    char name[64];
    uint8_t vessel_type;
    uint16_t bow_dim;
    uint16_t stern_dim;
    uint8_t port_dim;
    uint8_t star_dim;
    uint8_t fix;
    uint8_t draught;
    char dest[64];
    uint8_t dte;
};

// @LoggerMessage: AISR
// @Description: Raw AIS AVDIM messages contents, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_aivdmaivdo_sentence_layer
// @Field: TimeUS: Time since system startup
// @Field: num: count of fragments in the currently accumulating message
// @Field: total: fragment number of this sentence
// @Field: ID: sequential message ID for multi-sentence messages
// @Field: payload: data payload

// @LoggerMessage: AIS1
// @Description: Contents of 'position report' AIS message, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_types_1_2_and_3_position_report_class_a
// @Field: US: Time since system startup
// @Field: typ: Message Type
// @Field: rep: Repeat Indicator
// @Field: mmsi: MMSI
// @Field: nav: Navigation Status
// @Field: rot: Rate of Turn (ROT)
// @Field: sog: Speed Over Ground (SOG)
// @Field: pos: Position Accuracy
// @Field: lon: Longitude
// @Field: lat: Latitude
// @Field: cog: Course Over Ground (COG)
// @Field: hed: True Heading (HDG)
// @Field: sec: Time Stamp
// @Field: man: Maneuver Indicator
// @Field: raim: RAIM flag
// @Field: rad: Radio status

// @LoggerMessage: AIS4
// @Description: Contents of 'Base Station Report' AIS message, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_type_4_base_station_report
// @Field: US: Time since system startup
// @Field: rep: Repeat Indicator
// @Field: mmsi: MMSI
// @Field: year: Year (UTC)
// @Field: mth: Month (UTC)
// @Field: day: Day (UTC)
// @Field: h: Hour (UTC)
// @Field: m: Minute (UTC)
// @Field: s: Second (UTC)
// @Field: fix: Fix quality
// @Field: lon: Longitude
// @Field: lat: Latitude
// @Field: epfd: Type of EPFD
// @Field: raim: RAIM flag
// @Field: rad: Radio status

// @LoggerMessage: AIS5
// @Description: Contents of 'static and voyage related data' AIS message, see: https://gpsd.gitlab.io/gpsd/AIVDM.html#_type_5_static_and_voyage_related_data
// @Field: US: Time since system startup
// @Field: rep: Repeat Indicator
// @Field: mmsi: MMSI
// @Field: ver: AIS Version
// @Field: imo: IMO Number
// @Field: cal: Call Sign
// @Field: nam: Vessel Name
// @Field: typ: Ship Type
// @Field: bow: Dimension to Bow
// @Field: stn: Dimension to Stern
// @Field: prt: Dimension to Port
// @Field: str: Dimension to Starboard
// @Field: fix: Position Fix Type
// @Field: dght: Draught
// @Field: dst: Destination
// @Field: dte: DTE

#if !AP_AIS_ENABLED
#define LOG_STRUCTURE_FROM_AIS
#else
#define LOG_STRUCTURE_FROM_AIS \
    { LOG_AIS_RAW_MSG, sizeof(log_AIS_raw), \
      "AISR",  "QBBBZ", "TimeUS,num,total,ID,payload", "s----", "F0000" }, \
    { LOG_AIS_MSG1, sizeof(log_AIS_msg1), \
      "AIS1",  "QBBIBbHbLLHHBBbI", "US,typ,rep,mmsi,nav,rot,sog,pos,lon,lat,cog,hed,sec,man,raim,rad", "s---------------", "F000000000000000" }, \
    { LOG_AIS_MSG4, sizeof(log_AIS_msg4), \
      "AIS4",  "QBIHBBBBBBLLBBI", "US,rep,mmsi,year,mth,day,h,m,s,fix,lon,lat,epfd,raim,rad", "s--------------", "F00000000000000" }, \
    { LOG_AIS_MSG5, sizeof(log_AIS_msg5), \
      "AIS5",  "QBIBINZBHHBBBBZB", "US,rep,mmsi,ver,imo,cal,nam,typ,bow,stn,prt,str,fix,dght,dst,dte", "s-------mmmm-m--", "F------------A--" },
#endif
