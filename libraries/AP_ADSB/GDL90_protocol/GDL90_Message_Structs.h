/*
   Copyright (C) 2021  Kraus Hamdani Aerospace Inc. All rights reserved.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

  
  Author: GDL90/UCP protocol by uAvionix, 2021.
  Implemented by: Tom Pittenger
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum __attribute__((__packed__))
{
  GDL90_ID_HEARTBEAT                  = 0,
  GDL90_ID_OWNSHIP_REPORT             = 10,     // 0x0A
  GDL90_ID_OWNSHIP_GEOMETRIC_ALTITUDE = 11,     // 0x0B
  GDL90_ID_IDENTIFICATION             = 37,     // 0x25
  GDL90_ID_SENSOR_MESSAGE             = 40,     // 0x28
  GDL90_ID_TRANSPONDER_CONFIG         = 43,     // 0x2B
  GDL90_ID_MESSAGE_REQUEST            = 44,     // 0x2C
  GDL90_ID_TRANSPONDER_CONTROL        = 45,     // 0x2D
  GDL90_ID_GPS_DATA                   = 46,     // 0x2E
  GDL90_ID_TRANSPONDER_STATUS         = 47,     // 0x2F
} GDL90_MESSAGE_ID;

typedef enum  __attribute__((__packed__))
{
  ADSB_NIC_BARO_UNVERIFIED = 0, // Baro is Gilman bases, and not cross checked
  ADSB_NIC_BARO_VERIFIED = 1, // Baro is cross-checked, or not Gilman based
} ADSB_NIC_BARO; // Barometric Altitude Integrity Code

typedef enum __attribute__((__packed__))
{
  ADSB_AIRBORNE_SUBSONIC   = 0,
  ADSB_AIRBORNE_SUPERSONIC = 1,
  ADSB_ON_GROUND           = 2,
  // 3 Reserved
} ADSB_AIR_GROUND_STATE; // Determines how horizontal velocity fields are processed in UAT

typedef enum  __attribute__((__packed__))
{
  ADSB_EMERGENCY_NONE             = 0,
  ADSB_EMERGENCY_GENERAL          = 1,
  ADSB_EMERGENCY_MEDICAL          = 2,
  ADSB_EMERGENCY_MINIMUM_FUEL     = 3,
  ADSB_EMERGENCY_NO_COMMUNICATION = 4,
  ADSB_EMERGNECY_INTERFERENCE     = 5,
  ADSB_EMERGENCY_DOWNED_AIRCRAFT  = 6,
  ADSB_EMERGENCY_UAS_LOST_LINK    = 7,
  // 7 Reserved
} ADSB_EMERGENCY_STATUS;

#define GDL90_TRANSPONDER_CONTROL_VERSION (2)
#if GDL90_TRANSPONDER_CONTROL_VERSION == 1
typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  ADSB_NIC_BARO                 baroCrossChecked    : 1;
  ADSB_AIR_GROUND_STATE         airGroundState      : 2;
  uint8_t                       identActive         : 1;
  uint8_t                       modeAEnabled        : 1;
  uint8_t                       modeCEnabled        : 1;
  uint8_t                       modeSEnabled        : 1;
  uint8_t                       es1090TxEnabled     : 1;
  int32_t                       externalBaroAltitude_mm;
  uint16_t                      squawkCode;
  ADSB_EMERGENCY_STATUS         emergencyState;
  uint8_t                       callsign[8];
} GDL90_TRANSPONDER_CONTROL_MSG;
#elif GDL90_TRANSPONDER_CONTROL_VERSION == 2
typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  ADSB_NIC_BARO                 baroCrossChecked    : 1;
  ADSB_AIR_GROUND_STATE         airGroundState      : 2;
  uint8_t                       identActive         : 1;
  uint8_t                       modeAEnabled        : 1;
  uint8_t                       modeCEnabled        : 1;
  uint8_t                       modeSEnabled        : 1;
  uint8_t                       es1090TxEnabled     : 1;
  int32_t                       externalBaroAltitude_mm;
  uint16_t                      squawkCode;
  ADSB_EMERGENCY_STATUS         emergencyState;
  uint8_t                       callsign[8];
  uint8_t                       rfu                 : 7;
  uint8_t                       x_bit               : 1;
} GDL90_TRANSPONDER_CONTROL_MSG;
#endif

#define GDL90_TRANSPONDER_STATUS_VERSION (1) // Version 1 is the correct UCP format; version 3 is half-duplex and not used by the ping200x
#define GDL90_STATUS_MAX_ALTITUDE_FT (101338)
#define GDL90_STATUS_MIN_ALTITUDE_FT (-1000)
#if GDL90_TRANSPONDER_STATUS_VERSION == 1
typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  uint8_t                       rfu                   : 2;
  uint8_t                       x_bit                 : 1;
  uint8_t                       identActive           : 1;
  uint8_t                       modeAEnabled          : 1;
  uint8_t                       modeCEnabled          : 1;
  uint8_t                       modeSEnabled          : 1;
  uint8_t                       es1090TxEnabled       : 1;
  uint16_t                      modeARepliesPerSecond;
  uint16_t                      modecRepliesPerSecond;
  uint16_t                      modeSRepliesPerSecond;
  uint16_t                      squawkCode;
} GDL90_TRANSPONDER_STATUS_MSG;
#endif
#if GDL90_TRANSPONDER_STATUS_VERSION == 3
typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  uint8_t                       indicatingOnGround    : 1;
  uint8_t                       interrogatedSinceLast : 1;
  uint8_t                       fault                 : 1;
  uint8_t                       identActive           : 1;
  uint8_t                       modeAEnabled          : 1;
  uint8_t                       modeCEnabled          : 1;
  uint8_t                       modeSEnabled          : 1;
  uint8_t                       es1090TxEnabled       : 1;
  uint8_t                       latitude[3];
  uint8_t                       longitude[3];
  uint32_t                      track_Heading         : 8;
  uint32_t                      horizontalVelocity    :12;
  uint32_t                      altitude              :12;
  uint16_t                      squawkCode;
  uint8_t                       NIC                   : 4;
  uint8_t                       NACp                  : 4;
  uint8_t                       temperature;
  uint16_t                      crc;
} GDL90_TRANSPONDER_STATUS_MSG;
#endif


typedef struct __attribute__((__packed__))
{
  uint8_t HPLfdeActive    : 1;
  uint8_t fault           : 1;
  uint8_t HrdMagNorth     : 1;
  uint8_t reserved        : 5;
} GDL90_GPS_NAV_STATE;

typedef enum __attribute__((__packed__))
{
  GPS_FIX_NONE = 0,
  GPS_FIX_NO_FIX = 1,
  GPS_FIX_2D = 2,
  GPS_FIX_3D = 3,
  GPS_FIX_DIFFERENTIAL = 4,
  GPS_FIX_RTK = 5,
} GPS_FIX;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  uint32_t                      utcTime_s;  // Time since GPS epoch
  int32_t                       latitude_ddE7;
  int32_t                       longitude_ddE7;
  int32_t                       altitudeGnss_mm; // Height about WGS84 ellipsoid
  // Protection Limits. FD or SBAS-based depending on state bits
  uint32_t                      HPL_mm;
  uint32_t                      VPL_cm;
  // FOMS
  uint32_t                      horizontalFOM_mm;
  uint16_t                      verticalFOM_cm;
  uint16_t                      horizontalVelocityFOM_mmps;
  uint16_t                      verticalVelocityFOM_mmps;
  // Velocities
  int16_t                       verticalVelocity_cmps;
  int32_t                       northVelocity_mmps; // millimeter/s
  int32_t                       eastVelocity_mmps;
  // State
  GPS_FIX                       fixType;
  GDL90_GPS_NAV_STATE           navState;
  uint8_t                       satsUsed;
} GDL90_GPS_DATA_V2;
#define GDL90_GPS_DATA_VERSION  (2)

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  GDL90_MESSAGE_ID              reqMsgId;
} GDL90_TRANSPONDER_MESSAGE_REQUEST_V2;

typedef enum __attribute__((__packed__))
{
  GDL90_BARO_DATA_SOURCE_INTERNAL = 0,
  GDL90_BARO_DATA_SOURCE_EXTERNAL,
}GDL90_BARO_DATA_SOURCE;

typedef enum  __attribute__((__packed__))
{
  ADSB_SDA_UNKNOWN  = 0,
  ADSB_SDA_10_NEG3  = 1,
  ADSB_SDA_10_NEG5  = 2,
  ADSB_SDA_10_NEG7  = 3,
} ADSB_SDA; // System Design Assurance

typedef enum __attribute__((__packed__))
{
  ADSB_SIL_UNKNOWN  = 0,
  ADSB_SIL_10_NEG3  = 1,
  ADSB_SIL_10_NEG5  = 2,
  ADSB_SIL_10_NEG7  = 3,
} ADSB_SIL; // Source Integrity Level

typedef enum __attribute__((__packed__))
{
  ADSB_AV_LW_NO_DATA   = 0,
  ADSB_AV_LW_15M_23M   = 1,
  ADSB_AV_LW_25M_28P5M = 2,
  ADSB_AV_LW_25M_34M   = 3,
  ADSB_AV_LW_35M_33M   = 4,
  ADSB_AV_LW_35M_38M   = 5,
  ADSB_AV_LW_45M_39P5M = 6,
  ADSB_AV_LW_45M_45M   = 7,
  ADSB_AV_LW_55M_45M   = 8,
  ADSB_AV_LW_55M_52M   = 9,
  ADSB_AV_LW_65M_59P5M = 10,
  ADSB_AV_LW_65M_67M   = 11,
  ADSB_AV_LW_75M_72P5M = 12,
  ADSB_AV_LW_75M_80M   = 13,
  ADSB_AV_LW_85M_80M   = 14,
  ADSB_AV_LW_85M_90M   = 15,
} ADSB_AIRCRAFT_LENGTH_WIDTH;

typedef enum __attribute__((__packed__))
{
  ADSB_NOT_UAT_IN_CAPABLE     = 0,
  ADSB_UAT_IN_CAPABLE         = 1
} ADSB_UAT_IN_CAPABILITY;

typedef enum __attribute__((__packed__))
{
  ADSB_NOT_1090ES_IN_CAPABLE  = 0,
  ADSB_1090ES_IN_CAPABLE      = 1
} ADSB_1090ES_IN_CAPABILITY;

typedef enum __attribute__((__packed__))
{
  ADSB_GPS_LON_NO_DATA = 0,
  ADSB_GPS_LON_FROM_SENSOR = 1,
  // 2 - 31 valid values in 2 meter increments
} ADSB_GPS_LONGITUDINAL_OFFSET;

typedef enum __attribute__((__packed__))
{
  ADSB_GPS_LAT_NO_DATA = 0,
  ADSB_GPS_LAT_LEFT_2M = 1,
  ADSB_GPS_LAT_LEFT_4M = 2,
  ADSB_GPS_LAT_LEFT_6M = 3,
  ADSB_GPS_LAT_0M = 4,
  ADSB_GPS_LAT_RIGHT_2M = 5,
  ADSB_GPS_LAT_RIGHT_4M = 6,
  ADSB_GPS_LAT_RIGHT_6M = 7,
} ADSB_GPS_LATERAL_OFFSET;

typedef enum __attribute__((__packed__))
{
  ADSB_EMITTER_NO_INFO           = 0,
  ADSB_EMITTER_LIGHT             = 1,
  ADSB_EMITTER_SMALL             = 2,
  ADSB_EMITTER_LARGE             = 3,
  ADSB_EMITTER_HIGH_VORTEX_LARGE = 4,
  ADSB_EMITTER_HEAVY             = 5,
  ADSB_EMITTER_HIGHLY_MANUV      = 6,
  ADSB_EMITTER_ROTOCRAFT         = 7,
  // 8 Unassigned
  ADSB_EMITTER_GLIDER            = 9,
  ADSB_EMITTER_LIGHTER_AIR       = 10,
  ADSB_EMITTER_PARACHUTE         = 11,
  ADSB_EMITTER_ULTRA_LIGHT       = 12,
  // 13 Unassigned
  ADSB_EMITTER_UAV               = 14,
  ADSB_EMITTER_SPACE             = 15,
  // 16 Unassigned
  
  // Surface types
  ADSB_EMITTER_EMERGENCY_SURFACE = 17,
  ADSB_EMITTER_SERVICE_SURFACE   = 18,
  
  // Obstacle types
  ADSB_EMITTER_POINT_OBSTACLE    = 19,
  ADSB_EMITTER_CLUSTER_OBSTACLE  = 20,
  ADSB_EMITTER_LINE_OBSTACLE     = 21,  
  // 22 - 39 Reserved
} ADSB_EMITTER; // ADSB Emitter Category

typedef enum __attribute__((__packed__))
{
  PING_COM_1200_BAUD    = 0,
  PING_COM_2400_BAUD    = 1,
  PING_COM_4800_BAUD    = 2,
  PING_COM_9600_BAUD    = 3,
  PING_COM_19200_BAUD   = 4,
  PING_COM_38400_BAUD   = 5,
  PING_COM_57600_BAUD   = 6,
  PING_COM_115200_BAUD  = 7,
  PING_COM_921600_BAUD  = 8,
} PING_COM_RATE;

typedef enum __attribute__((__packed__))
{
  CONFIG_VALIDITY_ICAO                  = 1 << 0,
  CONFIG_VALIDITY_SIL                   = 1 << 1,
  CONFIG_VALIDITY_SDA                   = 1 << 2,
  CONFIG_VALIDITY_BARO_ALT_SOURCE       = 1 << 3,
  CONFIG_VALIDITY_AIRCRAFT_MAX_SPEED    = 1 << 4,
  CONFIG_VALIDITY_TEST_MODE             = 1 << 5,
  CONFIG_VALIDITY_ADSB_IN_CAP           = 1 << 6,
  CONFIG_VALIDITY_AIRCRAFT_LEN_WIDTH    = 1 << 7,
  CONFIG_VALIDITY_ANT_LAT_OFFSET        = 1 << 8,
  CONFIG_VALIDITY_ANT_LONG_OFFSET       = 1 << 9,
  CONFIG_VALIDITY_AIRCRAFT_REG          = 1 << 10,
  CONFIG_VALIDITY_AIRCRAFT_STALL_SPEED  = 1 << 11,
  CONFIG_VALIDITY_AIRCRAFT_EMITTER_TYPE = 1 << 12,
  CONFIG_VALIDITY_DEF_1090ES_TX_MODE    = 1 << 13,
  CONFIG_VALIDITY_DEF_MODES_REPLY_MODE  = 1 << 14,
  CONFIG_VALIDITY_DEF_MODEC_REPLY_MODE  = 1 << 15,
  CONFIG_VALIDITY_DEF_MODEA_REPLY_MODE  = 1 << 16,
  CONFIG_VALIDITY_SERIAL_BAUD_RATE      = 1 << 17,
  CONFIG_VALIDITY_DEF_MODEA_SQUAWK      = 1 << 18,
  CONFIG_VALIDITY_BARO_100              = 1 << 19,
  CONFIG_VALIDITY_IN_PROTOCOL           = 1 << 20,
  CONFIG_VALIDITY_OUT_PROTOCOL          = 1 << 21,
} CONFIG_VALIDITY;

typedef union __attribute__((__packed__))
{
  struct __attribute__((__packed__))
  {
    uint32_t icaoValid : 1;
    uint32_t silValid : 1;
    uint32_t sdaValid : 1;
    uint32_t baroAltSourceValid : 1;
    uint32_t aircraftMaxSpeedValid : 1;
    uint32_t testModeValid : 1;
    uint32_t adsbInCapValid : 1;
    uint32_t aircraftLenWidthValid : 1;
    uint32_t aircraftLatOffsetValid : 1;
    uint32_t aircraftLongOffsetValid : 1;
    uint32_t aircraftRegValid : 1;
    uint32_t aircraftStallSpeedValid : 1;
    uint32_t aircraftEmitterCatValid : 1;
    uint32_t default1090ExTxModeValid : 1;
    uint32_t defaultModeSReplyModeValid : 1;
    uint32_t defaultModeCReplyModeValid : 1;
    uint32_t defaultModeAReplyModeValid : 1;
    uint32_t serialBaudRateValid : 1;
    uint32_t defaultModeASquawkValid : 1;
    uint32_t baro100Valid : 1;
    uint32_t inProtocolValid : 1;
    uint32_t outProtocolValid : 1;
    uint32_t reserved : 10;
  };
  CONFIG_VALIDITY raw;
} CONFIG_VALIDITY_BITMASK;

typedef enum __attribute__((__packed__))
{
  PING_PROTOCOL_NONE          = 0,
  PING_PROTOCOL_MAVLINK       = 1 << 0,
  PING_PROTOCOL_UCP           = 1 << 1,
  PING_PROTOCOL_APOLLO        = 1 << 9,
  PING_PROTOCOL_UCP_HD        = 1 << 10,
} PING_PROTOCOL;

typedef union
{
  struct __attribute__((__packed__))
  {
    uint16_t mavlink : 1;
    uint16_t ucp : 1;
    uint16_t reserved1 : 7;
    uint16_t apollo : 1;
    uint16_t ucphd : 1;
    uint16_t reserved2 : 5;
  };
  PING_PROTOCOL raw;
} PING_PROTOCOL_MASK;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID              messageId;
  uint8_t                       version;
  uint8_t                       icaoAddress[3];
  uint8_t                       maxSpeed            : 3;
  GDL90_BARO_DATA_SOURCE        baroAltSource       : 1;
  ADSB_SDA                      SDA                 : 2;
  ADSB_SIL                      SIL                 : 2;
  ADSB_AIRCRAFT_LENGTH_WIDTH    lengthWidth         : 4;
  ADSB_1090ES_IN_CAPABILITY     es1090InCapable     : 1;
  ADSB_UAT_IN_CAPABILITY        uatInCapable        : 1;
  uint8_t                       testMode            : 2;
  ADSB_GPS_LONGITUDINAL_OFFSET  longitudinalOffset  : 5;
  ADSB_GPS_LATERAL_OFFSET       lateralOffset       : 3;
  uint8_t                       registration[8];
  uint16_t                      stallSpeed_cmps;
  ADSB_EMITTER                  emitterType;
  PING_COM_RATE                 baudRate            : 4;
  uint8_t                       modeAEnabled        : 1;
  uint8_t                       modeCEnabled        : 1;
  uint8_t                       modeSEnabled        : 1;
  uint8_t                       es1090TxEnabled     : 1;
  uint16_t                      defaultSquawk;
  CONFIG_VALIDITY_BITMASK       valdityBitmask;
  uint8_t                       rfu                 : 7;
  uint8_t                       baro100             : 1;
  PING_PROTOCOL_MASK            inProtocol;
  PING_PROTOCOL_MASK            outProtocol;
  uint16_t                      crc;
} GDL90_TRANSPONDER_CONFIG_MSG_V4_V5;


typedef struct __attribute__((__packed__))
{
  uint8_t           fwMajorVersion;
  uint8_t           fwMinorVersion;
  uint8_t           fwBuildVersion;
  uint8_t           hwId; // TODO Ugh should be 16 bits
  uint64_t          serialNumber;
} GDL90_DEVICE_ID;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID  messageId;
  uint8_t           protocolVersion;
  GDL90_DEVICE_ID   primary;
  GDL90_DEVICE_ID   secondary;
  uint8_t           primaryFWID;
  uint32_t          primaryCRC;
  uint8_t           secondaryFWID;
  uint32_t          secondaryCRC;
  uint8_t           primaryFwPartNumber[15];
  uint8_t           secondaryFwPartNumber[15];
  uint16_t          crc;
} GDL90_IDENTIFICATION_V3;
#define GDL90_IDENT_PROTOCOL_VERSION (3)


typedef struct __attribute__((__packed__))
{
  struct
  {
    uint8_t uatInitialized : 1;

    // GDL90 public spec defines next bit as reserved
    // uAvionix maps extra failure condition
    uint8_t functionFailureGnssDataFrequency : 1;

    uint8_t ratcs : 1;
    uint8_t gpsBatteryLow : 1;
    uint8_t addressType : 1;
    uint8_t ident : 1;
    uint8_t maintenanceRequired : 1;
    uint8_t gpsPositionValid : 1;
  } one;
  struct __attribute__((__packed__))
  {

    uint8_t utcOk : 1;

    // GDL90 public spec defines next four bits as reserved
    // uAvionix maps extra failure conditions
    uint8_t functionFailureGnssUnavailable : 1;
    uint8_t functionFailureGnssNo3dFix : 1;
    uint8_t functionFailureBroadcastMonitor : 1;
    uint8_t functionFailureTransmitSystem : 1;

    uint8_t csaNotAvailable : 1;
    uint8_t csaRequested : 1;
    uint8_t timestampMsb : 1;
  } two;
} GDL90_HEARTBEAT_STATUS;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID       messageId;
  GDL90_HEARTBEAT_STATUS status;
  uint16_t               timestamp;

  // Need to flip before TX
  union
  {
    struct __attribute__((__packed__))
    {
      uint16_t uatMessages : 10;
      uint16_t rfu : 1;
      uint16_t uplinkMessages : 5;
    };
    uint16_t             messageCount;
  };
  uint16_t               crc;
} GDL90_HEARTBEAT;

typedef enum __attribute__((__packed__))
{
  GDL90_ADDRESS_ADSB_ICAO,
  GDL90_ADDRESS_ADSB_SELF_ASSIGNED,
  GDL90_ADDRESS_TISB_ICAO,
  GDL90_ADDRESS_TISB_TRACK_ID,
  GDL90_ADDRESS_SURFACE,
  GDL90_ADDRESS_GROUND_BEACON,
} GDL90_ADDRESS_TYPE;

typedef enum __attribute__((__packed__))
{
  GDL90_NO_ALERT,
  GDL90_ALERT,
} GDL90_TRAFFIC_ALERT;

typedef enum __attribute__((__packed__))
{
  GDL90_MISC_INVALID,
  GDL90_MISC_TRUE_TRACK,
  GDL90_MISC_HEADING_MAGNETIC,
  GDL90_MISC_HEADING_TRUE,
} GDL90_MISC_TRACK_TYPE;

typedef enum __attribute__((__packed__))
{
  GDL90_MISC_REPORT_UPDATED,
  GDL90_MISC_REPORT_EXTRAPOLATED,
} GDL90_MISC_REPORT_TYPE;

typedef enum __attribute__((__packed__))
{
  GDL90_MISC_ON_GROUND,
  GDL90_MISC_AIRBORNE,
} GDL90_MISC_AG_STATE;

typedef union
{
  struct __attribute__((__packed__))
  {
    GDL90_MISC_TRACK_TYPE     track : 2;
    GDL90_MISC_REPORT_TYPE    reportType : 1;
    GDL90_MISC_AG_STATE       agState : 1;
  };
  uint8_t data;
} GDL90_MISCELLANEOUS;

typedef struct __attribute__((__packed__))
{
  GDL90_ADDRESS_TYPE addressType: 4;
  GDL90_TRAFFIC_ALERT trafficAlert : 4;

  uint8_t address[3];

  uint8_t latitude[3]; // 180 deg / 2^23
  uint8_t longitude[3]; // 180 deg / 2^23

  // Byte order must be flipped before TX
  union
  {
    struct __attribute__((__packed__))
    {
      uint16_t misc : 4;
      uint16_t altitude : 12;
    };
    uint16_t altitudeMisc;
  };

  uint8_t NACp : 4;
  uint8_t NIC : 4;

  // Byte order must be flipped before TX
  union
  {
    struct __attribute__((__packed__))
    {
      uint32_t heading : 8;
      uint32_t verticalVelocity : 12;
      uint32_t horizontalVelocity : 12;
    };
    uint32_t velocities;
  };

  uint8_t emitterCategory;
  uint8_t callsign[8];

  uint8_t rfu : 4;
  uint8_t emergencyCode : 4;
} GDL90_REPORT;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID messageId;
  GDL90_REPORT     report;
  uint16_t         crc;
} GDL90_OWNSHIP_REPORT;
typedef GDL90_OWNSHIP_REPORT GDL90_TRAFFIC_REPORT;

typedef enum __attribute__((__packed__))
{
  GDL90_NO_WARNING,
  GDL90_WARNING,
} GDL90_VERTICAL_WARNING;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID messageId;
  uint16_t geometricAltitude; // 5 ft resolution

  // Must be endian swapped before TX
  union
  {
    struct __attribute__((__packed__))
    {
      uint16_t verticalFOM : 15;
      GDL90_VERTICAL_WARNING verticalWarning : 1;
    };
    uint16_t veritcalMetrics;
  };
  uint16_t crc;
} GDL90_OWNSHIP_GEO_ALTITUDE;

typedef enum __attribute__((__packed__))
{
  GDL90_SENSOR_AHRS = 0,
  GDL90_SENSOR_BARO = 1,
  GDL90_SENSOR_CO = 2,
  GDL90_SENSOR_DEVICE = 3
} GDL90_SENSOR_TYPE;

typedef struct __attribute__((__packed__))
{
  GDL90_MESSAGE_ID  messageId;
  GDL90_SENSOR_TYPE sensorType;
  uint32_t          pressure_mbarE2;
  int32_t           pressureAlt_mm;
  int16_t           temperature_cE2;
  uint16_t          crc;
} GDL90_SENSOR_BARO_MESSAGE;

