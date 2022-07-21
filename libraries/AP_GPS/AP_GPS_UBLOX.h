/*
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
 */

//
//  u-blox UBX GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//  UBlox Lea6H protocol: http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_UBLOX_ENABLED

#include <AP_HAL/AP_HAL.h>

/*
 *  try to put a UBlox into binary mode. This is in two parts. 
 *
 * First we send a ubx binary message that enables the NAV_SOL message
 * at rate 1. Then we send a NMEA message to set the baud rate to our
 * desired rate. The reason for doing the NMEA message second is if we
 * send it first the second message will be ignored for a baud rate
 * change.
 * The reason we need the NAV_SOL rate message at all is some uBlox
 * modules are configured with all ubx binary messages off, which
 * would mean we would never detect it.

 * running a uBlox at less than 38400 will lead to packet
 * corruption, as we can't receive the packets in the 200ms
 * window for 5Hz fixes. The NMEA startup message should force
 * the uBlox into 230400 no matter what rate it is configured
 * for.
 */
#define UBLOX_SET_BINARY_115200 "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,115200,0*1C\r\n"

// a variant with 230400 baudrate
#define UBLOX_SET_BINARY_230400 "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,230400,0*1E\r\n"

// a variant with 460800 baudrate
#define UBLOX_SET_BINARY_460800 "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,460800,0*11\r\n"

#define UBLOX_RXM_RAW_LOGGING 1
#define UBLOX_MAX_RXM_RAW_SATS 22
#define UBLOX_MAX_RXM_RAWX_SATS 32
#define UBLOX_GNSS_SETTINGS 1
#ifndef UBLOX_TIM_TM2_LOGGING
    #define UBLOX_TIM_TM2_LOGGING (BOARD_FLASH_SIZE>1024)
#endif

#define UBLOX_MAX_GNSS_CONFIG_BLOCKS 7

#define UBX_TIMEGPS_VALID_WEEK_MASK 0x2

#define UBLOX_MAX_PORTS 6

#define RATE_POSLLH 1
#define RATE_STATUS 1
#define RATE_SOL 1
#define RATE_TIMEGPS 5
#define RATE_PVT 1
#define RATE_VELNED 1
#define RATE_DOP 1
#define RATE_HW 5
#define RATE_HW2 5
#define RATE_TIM_TM2 1

#define CONFIG_RATE_NAV      (1<<0)
#define CONFIG_RATE_POSLLH   (1<<1)
#define CONFIG_RATE_STATUS   (1<<2)
#define CONFIG_RATE_SOL      (1<<3)
#define CONFIG_RATE_VELNED   (1<<4)
#define CONFIG_RATE_DOP      (1<<5)
#define CONFIG_RATE_MON_HW   (1<<6)
#define CONFIG_RATE_MON_HW2  (1<<7)
#define CONFIG_RATE_RAW      (1<<8)
#define CONFIG_VERSION       (1<<9)
#define CONFIG_NAV_SETTINGS  (1<<10)
#define CONFIG_GNSS          (1<<11)
#define CONFIG_SBAS          (1<<12)
#define CONFIG_RATE_PVT      (1<<13)
#define CONFIG_TP5           (1<<14)
#define CONFIG_RATE_TIMEGPS  (1<<15)
#define CONFIG_TMODE_MODE    (1<<16)
#define CONFIG_RTK_MOVBASE   (1<<17)
#define CONFIG_TIM_TM2       (1<<18)
#define CONFIG_LAST          (1<<19) // this must always be the last bit

#define CONFIG_REQUIRED_INITIAL (CONFIG_RATE_NAV | CONFIG_RATE_POSLLH | CONFIG_RATE_STATUS | CONFIG_RATE_VELNED)

#define CONFIG_ALL (CONFIG_RATE_NAV | CONFIG_RATE_POSLLH | CONFIG_RATE_STATUS | CONFIG_RATE_SOL | CONFIG_RATE_VELNED \
                    | CONFIG_RATE_DOP | CONFIG_RATE_MON_HW | CONFIG_RATE_MON_HW2 | CONFIG_RATE_RAW | CONFIG_VERSION \
                    | CONFIG_NAV_SETTINGS | CONFIG_GNSS | CONFIG_SBAS)

//Configuration Sub-Sections
#define SAVE_CFG_IO     (1<<0)
#define SAVE_CFG_MSG    (1<<1)
#define SAVE_CFG_INF    (1<<2)
#define SAVE_CFG_NAV    (1<<3)
#define SAVE_CFG_RXM    (1<<4)
#define SAVE_CFG_RINV   (1<<9)
#define SAVE_CFG_ANT    (1<<10)
#define SAVE_CFG_ALL    (SAVE_CFG_IO|SAVE_CFG_MSG|SAVE_CFG_INF|SAVE_CFG_NAV|SAVE_CFG_RXM|SAVE_CFG_RINV|SAVE_CFG_ANT)

class RTCM3_Parser;

class AP_GPS_UBLOX : public AP_GPS_Backend
{
public:
    AP_GPS_UBLOX(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port, AP_GPS::GPS_Role role);
    ~AP_GPS_UBLOX() override;

    // Methods
    bool read() override;

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    static bool _detect(struct UBLOX_detect_state &state, uint8_t data);

    bool is_configured(void) const override {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        if (!gps._auto_config) {
            return true;
        } else {
            return !_unconfigured_messages;
        }
#else
        return true;
#endif // CONFIG_HAL_BOARD != HAL_BOARD_SITL
    }

    void broadcast_configuration_failure_reason(void) const override;
    void Write_AP_Logger_Log_Startup_messages() const override;

    // get the velocity lag, returns true if the driver is confident in the returned value
    bool get_lag(float &lag_sec) const override;

    const char *name() const override { return "u-blox"; }

    // support for retrieving RTCMv3 data from a moving baseline base
    bool get_RTCMV3(const uint8_t *&bytes, uint16_t &len) override;
    void clear_RTCMV3(void) override;

    // ublox specific healthy checks
    bool is_healthy(void) const override;
    
private:
    // u-blox UBX protocol essentials
    struct PACKED ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };
#if UBLOX_GNSS_SETTINGS
    struct PACKED ubx_cfg_gnss {
        uint8_t msgVer;
        uint8_t numTrkChHw;
        uint8_t numTrkChUse;
        uint8_t numConfigBlocks;
        PACKED struct configBlock {
            uint8_t gnssId;
            uint8_t resTrkCh;
            uint8_t maxTrkCh;
            uint8_t reserved1;
            uint32_t flags;
        } configBlock[UBLOX_MAX_GNSS_CONFIG_BLOCKS];
    };
#endif
    struct PACKED ubx_cfg_nav_rate {
        uint16_t measure_rate_ms;
        uint16_t nav_rate;
        uint16_t timeref;
    };
    struct PACKED ubx_cfg_msg {
        uint8_t msg_class;
        uint8_t msg_id;
    };
    struct PACKED ubx_cfg_msg_rate {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rate;
    };
    struct PACKED ubx_cfg_msg_rate_6 {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rates[6];
    };
    struct PACKED ubx_cfg_nav_settings {
        uint16_t mask;
        uint8_t dynModel;
        uint8_t fixMode;
        int32_t fixedAlt;
        uint32_t fixedAltVar;
        int8_t minElev;
        uint8_t drLimit;
        uint16_t pDop;
        uint16_t tDop;
        uint16_t pAcc;
        uint16_t tAcc;
        uint8_t staticHoldThresh;
        uint8_t res1;
        uint32_t res2;
        uint32_t res3;
        uint32_t res4;
    };
    struct PACKED ubx_cfg_tp5 {
        uint8_t tpIdx;
        uint8_t version;
        uint8_t reserved1[2];
        int16_t antCableDelay;
        int16_t rfGroupDelay;
        uint32_t freqPeriod;
        uint32_t freqPeriodLock;
        uint32_t pulseLenRatio;
        uint32_t pulseLenRatioLock;
        int32_t userConfigDelay;
        uint32_t flags;
    };
    struct PACKED ubx_cfg_prt {
        uint8_t portID;
    };
    struct PACKED ubx_cfg_sbas {
        uint8_t mode;
        uint8_t usage;
        uint8_t maxSBAS;
        uint8_t scanmode2;
        uint32_t scanmode1;
    };
    // F9 config keys
    enum class ConfigKey : uint32_t {
        TMODE_MODE = 0x20030001,
        CFG_RATE_MEAS                   = 0x30210001,

        CFG_UART1_BAUDRATE              = 0x40520001,
        CFG_UART1_ENABLED               = 0x10520005,
        CFG_UART1INPROT_UBX             = 0x10730001,
        CFG_UART1INPROT_NMEA            = 0x10730002,
        CFG_UART1INPROT_RTCM3X          = 0x10730004,
        CFG_UART1OUTPROT_UBX            = 0x10740001,
        CFG_UART1OUTPROT_NMEA           = 0x10740002,
        CFG_UART1OUTPROT_RTCM3X         = 0x10740004,

        CFG_UART2_BAUDRATE              = 0x40530001,
        CFG_UART2_ENABLED               = 0x10530005,
        CFG_UART2INPROT_UBX             = 0x10750001,
        CFG_UART2INPROT_NMEA            = 0x10750002,
        CFG_UART2INPROT_RTCM3X          = 0x10750004,
        CFG_UART2OUTPROT_UBX            = 0x10760001,
        CFG_UART2OUTPROT_NMEA           = 0x10760002,
        CFG_UART2OUTPROT_RTCM3X         = 0x10760004,

        MSGOUT_RTCM_3X_TYPE4072_0_UART1 = 0x209102ff,
        MSGOUT_RTCM_3X_TYPE4072_1_UART1 = 0x20910382,
        MSGOUT_RTCM_3X_TYPE1077_UART1   = 0x209102cd,
        MSGOUT_RTCM_3X_TYPE1087_UART1   = 0x209102d2,
        MSGOUT_RTCM_3X_TYPE1097_UART1   = 0x20910319,
        MSGOUT_RTCM_3X_TYPE1127_UART1   = 0x209102d7,
        MSGOUT_RTCM_3X_TYPE1230_UART1   = 0x20910304,
        MSGOUT_UBX_NAV_RELPOSNED_UART1  = 0x2091008e,

        MSGOUT_RTCM_3X_TYPE4072_0_UART2 = 0x20910300,
        MSGOUT_RTCM_3X_TYPE4072_1_UART2 = 0x20910383,
        MSGOUT_RTCM_3X_TYPE1077_UART2   = 0x209102ce,
        MSGOUT_RTCM_3X_TYPE1087_UART2   = 0x209102d3,
        MSGOUT_RTCM_3X_TYPE1097_UART2   = 0x2091031a,
        MSGOUT_RTCM_3X_TYPE1127_UART2   = 0x209102d8,
        MSGOUT_RTCM_3X_TYPE1230_UART2   = 0x20910305,
        MSGOUT_UBX_NAV_RELPOSNED_UART2  = 0x2091008f,
    };
    struct PACKED ubx_cfg_valset {
        uint8_t version;
        uint8_t layers;
        uint8_t transaction;
        uint8_t reserved[1];
        uint32_t key;
    };
    struct PACKED ubx_cfg_valget {
        uint8_t version;
        uint8_t layers;
        uint8_t reserved[2];
        // variable length data, check buffer length
    };
    struct PACKED ubx_nav_posllh {
        uint32_t itow;                                  // GPS msToW
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    };
    struct PACKED ubx_nav_status {
        uint32_t itow;                                  // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;                                // milliseconds
    };
    struct PACKED ubx_nav_dop {
        uint32_t itow;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    };
    struct PACKED ubx_nav_solution {
        uint32_t itow;
        int32_t time_nsec;
        uint16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    };
    struct PACKED ubx_nav_pvt {
        uint32_t itow; 
        uint16_t year; 
        uint8_t month, day, hour, min, sec; 
        uint8_t valid; 
        uint32_t t_acc; 
        int32_t nano; 
        uint8_t fix_type; 
        uint8_t flags; 
        uint8_t flags2; 
        uint8_t num_sv; 
        int32_t lon, lat; 
        int32_t h_ellipsoid, h_msl;
        uint32_t h_acc, v_acc; 
        int32_t velN, velE, velD, gspeed; 
        int32_t head_mot; 
        uint32_t s_acc; 
        uint32_t head_acc; 
        uint16_t p_dop; 
        uint8_t flags3;
        uint8_t reserved1[5];
        int32_t headVeh;
        int16_t magDec;
        uint16_t magAcc;
    };
    struct PACKED ubx_nav_relposned {
        uint8_t version;
        uint8_t reserved1;
        uint16_t refStationId;
        uint32_t iTOW;
        int32_t relPosN;
        int32_t relPosE;
        int32_t relPosD;
        int32_t relPosLength;
        int32_t relPosHeading;
        uint8_t reserved2[4];
        int8_t relPosHPN;
        int8_t relPosHPE;
        int8_t relPosHPD;
        int8_t relPosHPLength;
        uint32_t accN;
        uint32_t accE;
        uint32_t accD;
        uint32_t accLength;
        uint32_t accHeading;
        uint8_t reserved3[4];
        uint32_t flags;
    };

    struct PACKED ubx_nav_velned {
        uint32_t itow;                                  // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
    };

    struct PACKED ubx_nav_timegps {
        uint32_t itow;
        int32_t ftow;
        uint16_t week;
        int8_t leapS;
        uint8_t valid; //  leapsvalid | weekvalid | tow valid;
        uint32_t tAcc;
    };

    // Lea6 uses a 60 byte message
    struct PACKED ubx_mon_hw_60 {
        uint32_t pinSel;
        uint32_t pinBank;
        uint32_t pinDir;
        uint32_t pinVal;
        uint16_t noisePerMS;
        uint16_t agcCnt;
        uint8_t aStatus;
        uint8_t aPower;
        uint8_t flags;
        uint8_t reserved1;
        uint32_t usedMask;
        uint8_t VP[17];
        uint8_t jamInd;
        uint16_t reserved3;
        uint32_t pinIrq;
        uint32_t pullH;
        uint32_t pullL;
    };
    // Neo7 uses a 68 byte message
    struct PACKED ubx_mon_hw_68 {
        uint32_t pinSel;
        uint32_t pinBank;
        uint32_t pinDir;
        uint32_t pinVal;
        uint16_t noisePerMS;
        uint16_t agcCnt;
        uint8_t aStatus;
        uint8_t aPower;
        uint8_t flags;
        uint8_t reserved1;
        uint32_t usedMask;
        uint8_t VP[25];
        uint8_t jamInd;
        uint16_t reserved3;
        uint32_t pinIrq;
        uint32_t pullH;
        uint32_t pullL;
    };
    struct PACKED ubx_mon_hw2 {
        int8_t ofsI;
        uint8_t magI;
        int8_t ofsQ;
        uint8_t magQ;
        uint8_t cfgSource;
        uint8_t reserved0[3];
        uint32_t lowLevCfg;
        uint32_t reserved1[2];
        uint32_t postStatus;
        uint32_t reserved2;
    };
    struct PACKED ubx_mon_ver {
        char swVersion[30];
        char hwVersion[10];
        char extension; // extensions are not enabled
    };
    struct PACKED ubx_nav_svinfo_header {
        uint32_t itow;
        uint8_t numCh;
        uint8_t globalFlags;
        uint16_t reserved;
    };
#if UBLOX_RXM_RAW_LOGGING
    struct PACKED ubx_rxm_raw {
        int32_t iTOW;
        int16_t week;
        uint8_t numSV;
        uint8_t reserved1;
        struct ubx_rxm_raw_sv {
            double cpMes;
            double prMes;
            float doMes;
            uint8_t sv;
            int8_t mesQI;
            int8_t cno;
            uint8_t lli;
        } svinfo[UBLOX_MAX_RXM_RAW_SATS];
    };
    struct PACKED ubx_rxm_rawx {
        double rcvTow;
        uint16_t week;
        int8_t leapS;
        uint8_t numMeas;
        uint8_t recStat;
        uint8_t reserved1[3];
        PACKED struct ubx_rxm_rawx_sv {
            double prMes;
            double cpMes;
            float doMes;
            uint8_t gnssId;
            uint8_t svId;
            uint8_t reserved2;
            uint8_t freqId;
            uint16_t locktime;
            uint8_t cno;
            uint8_t prStdev;
            uint8_t cpStdev;
            uint8_t doStdev;
            uint8_t trkStat;
            uint8_t reserved3;
        } svinfo[UBLOX_MAX_RXM_RAWX_SATS];
    };
#endif

    struct PACKED ubx_ack_ack {
        uint8_t clsID;
        uint8_t msgID;
    };


    struct PACKED ubx_cfg_cfg {
        uint32_t clearMask;
        uint32_t saveMask;
        uint32_t loadMask;
    };

    struct PACKED ubx_tim_tm2 {
        uint8_t ch;
        uint8_t flags;
        uint16_t count;
        uint16_t wnR;
        uint16_t wnF;
        uint32_t towMsR;
        uint32_t towSubMsR;
        uint32_t towMsF;
        uint32_t towSubMsF;
        uint32_t accEst;
    };

    // Receive buffer
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_dop dop;
        ubx_nav_solution solution;
        ubx_nav_pvt pvt;
        ubx_nav_timegps timegps;
        ubx_nav_velned velned;
        ubx_cfg_msg_rate msg_rate;
        ubx_cfg_msg_rate_6 msg_rate_6;
        ubx_cfg_nav_settings nav_settings;
        ubx_cfg_nav_rate nav_rate;
        ubx_cfg_prt prt;
        ubx_mon_hw_60 mon_hw_60;
        ubx_mon_hw_68 mon_hw_68;
        ubx_mon_hw2 mon_hw2;
        ubx_mon_ver mon_ver;
        ubx_cfg_tp5 nav_tp5;
#if UBLOX_GNSS_SETTINGS
        ubx_cfg_gnss gnss;
#endif
        ubx_cfg_sbas sbas;
        ubx_cfg_valget valget;
        ubx_nav_svinfo_header svinfo_header;
        ubx_nav_relposned relposned;
#if UBLOX_RXM_RAW_LOGGING
        ubx_rxm_raw rxm_raw;
        ubx_rxm_rawx rxm_rawx;
#endif
        ubx_ack_ack ack;
        ubx_tim_tm2 tim_tm2;
    } _buffer;

    enum class RELPOSNED {
        gnssFixOK          = 1U << 0,
        diffSoln           = 1U << 1,
        relPosValid        = 1U << 2,
        carrSolnFloat      = 1U << 3,

        carrSolnFixed      = 1U << 4,
        isMoving           = 1U << 5,
        refPosMiss         = 1U << 6,
        refObsMiss         = 1U << 7,

        relPosHeadingValid = 1U << 8,
        relPosNormalized   = 1U << 9
    };

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
        CLASS_MON = 0x0A,
        CLASS_RXM = 0x02,
        CLASS_TIM = 0x0d,
        MSG_ACK_NACK = 0x00,
        MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_DOP = 0x4,
        MSG_SOL = 0x6,
        MSG_PVT = 0x7,
        MSG_TIMEGPS = 0x20,
        MSG_RELPOSNED = 0x3c,
        MSG_VELNED = 0x12,
        MSG_CFG_CFG = 0x09,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_MSG = 0x01,
        MSG_CFG_NAV_SETTINGS = 0x24,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_SBAS = 0x16,
        MSG_CFG_GNSS = 0x3E,
        MSG_CFG_TP5 = 0x31,
        MSG_CFG_VALSET = 0x8A,
        MSG_CFG_VALGET = 0x8B,
        MSG_MON_HW = 0x09,
        MSG_MON_HW2 = 0x0B,
        MSG_MON_VER = 0x04,
        MSG_NAV_SVINFO = 0x30,
        MSG_RXM_RAW = 0x10,
        MSG_RXM_RAWX = 0x15,
        MSG_TIM_TM2 = 0x03
    };
    enum ubx_gnss_identifier {
        GNSS_GPS     = 0x00,
        GNSS_SBAS    = 0x01,
        GNSS_GALILEO = 0x02,
        GNSS_BEIDOU  = 0x03,
        GNSS_IMES    = 0x04,
        GNSS_QZSS    = 0x05,
        GNSS_GLONASS = 0x06
    };
    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1,
        NAV_STATUS_DGPS_USED = 2
    };
    enum ubx_hardware_version {
        ANTARIS = 0,
        UBLOX_5,
        UBLOX_6,
        UBLOX_7,
        UBLOX_M8,
        UBLOX_F9 = 0x80, // comes from MON_VER hwVersion/swVersion strings
        UBLOX_M9 = 0x81, // comes from MON_VER hwVersion/swVersion strings
        UBLOX_UNKNOWN_HARDWARE_GENERATION = 0xff // not in the ublox spec used for
                                                 // flagging state in the driver
    };

    enum config_step {
        STEP_PVT = 0,
        STEP_NAV_RATE, // poll NAV rate
        STEP_SOL,
        STEP_PORT,
        STEP_STATUS,
        STEP_POSLLH,
        STEP_VELNED,
        STEP_TIMEGPS,
        STEP_POLL_SVINFO, // poll svinfo
        STEP_POLL_SBAS, // poll SBAS
        STEP_POLL_NAV, // poll NAV settings
        STEP_POLL_GNSS, // poll GNSS
        STEP_POLL_TP5, // poll TP5
        STEP_TMODE, // set TMODE-MODE
        STEP_DOP,
        STEP_MON_HW,
        STEP_MON_HW2,
        STEP_RAW,
        STEP_RAWX,
        STEP_VERSION,
        STEP_RTK_MOVBASE, // setup moving baseline
        STEP_TIM_TM2,
        STEP_LAST
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _msg_id;
    uint16_t        _payload_length;
    uint16_t        _payload_counter;

    uint8_t         _class;
    bool            _cfg_saved;

    uint32_t        _last_vel_time;
    uint32_t        _last_pos_time;
    uint32_t        _last_cfg_sent_time;
    uint8_t         _num_cfg_save_tries;
    uint32_t        _last_config_time;
    uint16_t        _delay_time;
    uint8_t         _next_message;
    uint8_t         _ublox_port;
    bool            _have_version;
    struct ubx_mon_ver _version;
    uint32_t        _unconfigured_messages;
    uint8_t         _hardware_generation;
    uint32_t        _last_pvt_itow;
    uint32_t        _last_relposned_itow;
    uint32_t        _last_relposned_ms;

    // the role set from GPS_TYPE
    AP_GPS::GPS_Role role;

    // do we have new position information?
    bool            _new_position:1;
    // do we have new speed information?
    bool            _new_speed:1;

    uint8_t         _disable_counter;

    // Buffer parse & GPS state update
    bool        _parse_gps();

    // used to update fix between status and position packets
    AP_GPS::GPS_Status next_fix;

    bool _cfg_needs_save;

    bool noReceivedHdop;
    
    bool havePvtMsg;

    bool        _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    bool        _configure_valset(ConfigKey key, const void *value);
    bool        _configure_valget(ConfigKey key);
    void        _configure_rate(void);
    void        _configure_sbas(bool enable);
    void        _update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b);
    bool        _send_message(uint8_t msg_class, uint8_t msg_id, const void *msg, uint16_t size);
    void	send_next_rate_update(void);
    bool        _request_message_rate(uint8_t msg_class, uint8_t msg_id);
    void        _request_next_config(void);
    void        _request_port(void);
    void        _request_version(void);
    void        _save_cfg(void);
    void        _verify_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    void        _check_new_itow(uint32_t itow);

    void unexpected_message(void);
    void log_mon_hw(void);
    void log_mon_hw2(void);
    void log_tim_tm2(void);
    void log_rxm_raw(const struct ubx_rxm_raw &raw);
    void log_rxm_rawx(const struct ubx_rxm_rawx &raw);

#if GPS_MOVING_BASELINE
    // see if we should use uart2 for moving baseline config
    bool mb_use_uart2(void) const {
        return option_set(AP_GPS::DriverOptions::UBX_MBUseUart2)?true:false;
    }
#endif

    // structure for list of config key/value pairs for
    // specific configurations
    struct PACKED config_list {
        ConfigKey key;
        // support up to 4 byte values, assumes little-endian
        uint32_t value;
    };

    // return size of a config key payload
    uint8_t config_key_size(ConfigKey key) const;

    // configure a set of config key/value pairs. The unconfig_bit corresponds to
    // a bit in _unconfigured_messages
    bool _configure_config_set(const config_list *list, uint8_t count, uint32_t unconfig_bit);

    // find index in active_config list
    int8_t find_active_config_index(ConfigKey key) const;

    // return true if GPS is capable of F9 config
    bool supports_F9_config(void) const;

    uint8_t _pps_freq = 1;
#ifdef HAL_GPIO_PPS
    void pps_interrupt(uint8_t pin, bool high, uint32_t timestamp_us);
    void set_pps_desired_freq(uint8_t freq) override;
#endif

#if GPS_MOVING_BASELINE
    // config for moving baseline base
    static const config_list config_MB_Base_uart1[];
    static const config_list config_MB_Base_uart2[];

    // config for moving baseline rover
    static const config_list config_MB_Rover_uart1[];
    static const config_list config_MB_Rover_uart2[];

    // status of active configuration for a role
    struct {
        const config_list *list;
        uint8_t count;
        uint32_t done_mask;
        uint32_t unconfig_bit;
    } active_config;

    // RTCM3 parser for when in moving baseline base mode
    RTCM3_Parser *rtcm3_parser;
#endif // GPS_MOVING_BASELINE
};

#endif
