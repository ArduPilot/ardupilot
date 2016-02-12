// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#ifndef __AP_GPS_UBLOX_H__
#define __AP_GPS_UBLOX_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"

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
 */
#define UBLOX_SET_BINARY "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,38400,0*26\r\n"
#define UBLOX_SET_BINARY_RAW_BAUD "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,115200,0*1E\r\n"

#define UBLOX_RXM_RAW_LOGGING 1
#define UBLOX_MAX_RXM_RAW_SATS 22
#define UBLOX_MAX_RXM_RAWX_SATS 32
#define UBLOX_GNSS_SETTINGS 1

#define UBLOX_MAX_GNSS_CONFIG_BLOCKS 7
#define UBX_MSG_TYPES 2

#define UBLOX_MAX_PORTS 6
#define MEASURE_RATE 200

#define RATE_POSLLH 1
#define RATE_STATUS 1
#define RATE_SOL 1
#define RATE_VELNED 1
#define RATE_DOP 1
#define RATE_HW 5
#define RATE_HW2 5

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

class AP_GPS_UBLOX : public AP_GPS_Backend
{
public:
	AP_GPS_UBLOX(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // Methods
    bool read();

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_DGPS; }

    static bool _detect(struct UBLOX_detect_state &state, uint8_t data);

    bool is_configured(void) {
        if (!gps._auto_config) {
            return true;
        } else {
            return !_unconfigured_messages;
        }
    }

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
    struct PACKED ubx_nav_posllh {
        uint32_t time;                                  // GPS msToW
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    };
    struct PACKED ubx_nav_status {
        uint32_t time;                                  // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;                                // milliseconds
    };
    struct PACKED ubx_nav_dop {
        uint32_t time;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    };
    struct PACKED ubx_nav_solution {
        uint32_t time;
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
    struct PACKED ubx_nav_velned {
        uint32_t time;                                  // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
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

    // Receive buffer
    union PACKED {
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_dop dop;
        ubx_nav_solution solution;
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
#if UBLOX_GNSS_SETTINGS
        ubx_cfg_gnss gnss;
#endif
        ubx_cfg_sbas sbas;
        ubx_nav_svinfo_header svinfo_header;
#if UBLOX_RXM_RAW_LOGGING
        ubx_rxm_raw rxm_raw;
        ubx_rxm_rawx rxm_rawx;
#endif
        ubx_ack_ack ack;
        uint8_t bytes[];
    } _buffer;

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
        CLASS_MON = 0x0A,
        CLASS_RXM = 0x02,
        MSG_ACK_NACK = 0x00,
        MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_DOP = 0x4,
        MSG_SOL = 0x6,
        MSG_VELNED = 0x12,
        MSG_CFG_CFG = 0x09,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_MSG = 0x01,
        MSG_CFG_NAV_SETTINGS = 0x24,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_SBAS = 0x16,
        MSG_CFG_GNSS = 0x3E,
        MSG_MON_HW = 0x09,
        MSG_MON_HW2 = 0x0B,
        MSG_MON_VER = 0x04,
        MSG_NAV_SVINFO = 0x30,
        MSG_RXM_RAW = 0x10,
        MSG_RXM_RAWX = 0x15
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
        UBLOX_M8
    };

    enum config_step {
        STEP_RATE_NAV = 0,
        STEP_RATE_POSLLH,
        STEP_RATE_VELNED,
        STEP_PORT,
        STEP_POLL_SVINFO,
        STEP_POLL_SBAS,
        STEP_POLL_NAV,
        STEP_POLL_GNSS,
        STEP_NAV_RATE,
        STEP_POSLLH,
        STEP_STATUS,
        STEP_SOL,
        STEP_VELNED,
        STEP_DOP,
        STEP_MON_HW,
        STEP_MON_HW2,
        STEP_RAW,
        STEP_RAWX,
        STEP_VERSION,
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
    uint32_t        _unconfigured_messages;
    uint8_t         _hardware_generation;


    // do we have new position information?
    bool            _new_position:1;
    // do we have new speed information?
    bool            _new_speed:1;

    uint8_t         _disable_counter;

    // Buffer parse & GPS state update
    bool        _parse_gps();

    // used to update fix between status and position packets
    AP_GPS::GPS_Status next_fix;

    uint32_t _last_5hz_time;

    bool _cfg_needs_save;

    bool noReceivedHdop;

    bool        _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    void        _configure_rate(void);
    void        _configure_sbas(bool enable);
    void        _update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b);
    void        _send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size);
    void	send_next_rate_update(void);
    bool        _request_message_rate(uint8_t msg_class, uint8_t msg_id);
    void        _request_navigation_rate(void);
    void        _request_next_config(void);
    void        _request_port(void);
    void        _request_version(void);
    void        _save_cfg(void);
    void        _verify_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

    void unexpected_message(void);
    void write_logging_headers(void);
    void log_mon_hw(void);
    void log_mon_hw2(void);
    void log_mon_ver(void);
    void log_rxm_raw(const struct ubx_rxm_raw &raw);
    void log_rxm_rawx(const struct ubx_rxm_rawx &raw);

    // Calculates the correct log message ID based on what GPS instance is being logged
    uint8_t _ubx_msg_log_index(uint8_t ubx_msg) {
        return (uint8_t)(ubx_msg + (state.instance * UBX_MSG_TYPES));
    }
};

#endif // __AP_GPS_UBLOX_H__
