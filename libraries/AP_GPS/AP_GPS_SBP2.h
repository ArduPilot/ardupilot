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
//  Swift Navigation SBP GPS driver for ArduPilot.
//	Code by Niels Joubert
//
//  Swift Binary Protocol format: http://docs.swift-nav.com/
//
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_SBP2_ENABLED
class AP_GPS_SBP2 : public AP_GPS_Backend
{
public:
    AP_GPS_SBP2(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    void inject_data(const uint8_t *data, uint16_t len) override;

    static bool _detect(struct SBP2_detect_state &state, uint8_t data);

    const char *name() const override { return "SBP2"; }

private:

   // ************************************************************************
    // Swift Navigation SBP protocol types and definitions
    // ************************************************************************

    struct sbp_parser_state_t {
      enum {
        WAITING = 0,
        GET_TYPE = 1,
        GET_SENDER = 2,
        GET_LEN = 3,
        GET_MSG = 4,
        GET_CRC = 5
      } state:8;
      uint16_t msg_type;
      uint16_t sender_id;
      uint16_t crc;
      uint8_t msg_len;
      uint8_t n_read;
      uint8_t msg_buff[256];
    } parser_state;

    static const uint8_t SBP_PREAMBLE = 0x55;

    // Message types supported by this driver
    static const uint16_t SBP_STARTUP_MSGTYPE        = 0xFF00;
    static const uint16_t SBP_HEARTBEAT_MSGTYPE      = 0xFFFF;
    static const uint16_t SBP_GPS_TIME_MSGTYPE       = 0x0102;
    static const uint16_t SBP_DOPS_MSGTYPE           = 0x0208;
    static const uint16_t SBP_POS_ECEF_MSGTYPE       = 0x0209;
    static const uint16_t SBP_POS_LLH_MSGTYPE        = 0x020A;
    static const uint16_t SBP_BASELINE_ECEF_MSGTYPE  = 0x020B;
    static const uint16_t SBP_BASELINE_NED_MSGTYPE   = 0x020C;
    static const uint16_t SBP_VEL_ECEF_MSGTYPE       = 0x020D;
    static const uint16_t SBP_VEL_NED_MSGTYPE        = 0x020E;
    static const uint16_t SBP_TRACKING_STATE_MSGTYPE = 0x0013;
    static const uint16_t SBP_IAR_STATE_MSGTYPE      = 0x0019;
    static const uint16_t SBP_EXT_EVENT_MSGTYPE      = 0x0101;

    // Heartbeat
    struct PACKED sbp_heartbeat_t {
        bool sys_error : 1;
        bool io_error : 1;
        bool nap_error : 1;
        uint8_t res : 5;
        uint8_t protocol_minor : 8;
        uint8_t protocol_major : 8;
        uint8_t res2 : 6;
        bool ext_antenna_short : 1;
        bool ext_antenna : 1;
    }; // 4 bytes

    // GPS Time
    struct PACKED sbp_gps_time_t {
        uint16_t wn;     //< GPS week number (unit: weeks)
        uint32_t tow;    //< GPS Time of Week rounded to the nearest ms (unit: ms)
        int32_t ns;      //< Nanosecond remainder of rounded tow (unit: ns)
        struct PACKED flags {
            uint8_t time_src:3;  //< Fix mode (0: invalid, 1: GNSS Solution, 2: Propagated
            uint8_t res:5;       //< Reserved
        } flags;
    }; // 11 bytes

    // Dilution of Precision
    struct PACKED sbp_dops_t {
        uint32_t tow;    //< GPS Time of Week (unit: ms)
        uint16_t gdop;   //< Geometric Dilution of Precision (unit: 0.01)
        uint16_t pdop;   //< Position Dilution of Precision (unit: 0.01)
        uint16_t tdop;   //< Time Dilution of Precision (unit: 0.01)
        uint16_t hdop;   //< Horizontal Dilution of Precision (unit: 0.01)
        uint16_t vdop;   //< Vertical Dilution of Precision (unit: 0.01)
        struct PACKED flags {
            uint8_t fix_mode:3;  //< Fix mode (0: invalid, 1: SPP, 2: DGNSS, 3: Float RTX, 4: Fixed RTX, 5: Undefined, 6: SBAS Position
            uint8_t res:4;       //< Reserved
            bool raim_repair:1;  //< Solution from RAIM?
        } flags;
    }; // 15 bytes

    // Geodetic position solution.
    struct PACKED sbp_pos_llh_t {
        uint32_t tow;         //< GPS Time of Week (unit: ms)
        double lat;           //< Latitude (unit: degrees)
        double lon;           //< Longitude (unit: degrees)
        double height;        //< Height (unit: meters)
        uint16_t h_accuracy;  //< Horizontal position accuracy estimate (unit: mm)
        uint16_t v_accuracy;  //< Vertical position accuracy estimate (unit: mm)
        uint8_t n_sats;       //< Number of satellites used in solution
        struct PACKED flags {
            uint8_t fix_mode:3;  //< Fix mode (0: invalid, 1: SPP, 2: DGNSS, 3: Float RTX, 4: Fixed RTX, 5: Dead Reckoning, 6: SBAS Position
            uint8_t ins_mode:2;  //< Inertial navigation mode (0: none, 1: INS used)
            uint8_t res:3;       //< Reserved
        } flags;
    }; // 34 bytes

    // Velocity in NED Velocity in local North East Down (NED) coordinates.
    struct PACKED sbp_vel_ned_t {
        uint32_t tow;          //< GPS Time of Week (unit: ms)
        int32_t n;             //< Velocity North coordinate (unit: mm/s)
        int32_t e;             //< Velocity East coordinate  (unit: mm/s)
        int32_t d;             //< Velocity Down coordinate  (unit: mm/s)
        uint16_t h_accuracy;   //< Horizontal velocity accuracy estimate (unit: mm/s)
        uint16_t v_accuracy;   //< Vertical velocity accuracy estimate (unit: mm/s)
        uint8_t n_sats;        //< Number of satellites used in solution
        struct PACKED flags {
            uint8_t vel_mode:3;  //< Velocity mode (0: Invalid, 1: Measured Doppler derived, 2: Computed Doppler derived, 3: Dead reckoning)
            uint8_t ins_mode:2;  //< Inertial navigation mode (0: none, 1: INS used)
            uint8_t res:3;       //< Reserved
        } flags;
    }; // 22 bytes

    // Messages reporting accurately-timestamped external events, e.g. camera shutter time.
    struct PACKED sbp_ext_event_t {
        uint16_t wn;           //< GPS week number (unit: weeks)
        uint32_t tow;          //< GPS Time of Week (unit: ms)
        int32_t ns_residual;   //< Nanosecond residual of millisecond-rounded TOW (ranges from -500000 to 500000)
        struct PACKED flags {
            uint8_t level:1;       //< New level of pin values (0: Low (falling edge), 1: High (rising edge))
            uint8_t quality:1;     //< Time quality values (0: Unknown - don't have nav solution, 1: Good (< 1 microsecond))
            uint8_t res:6;         //< Reserved
        } flags;
        uint8_t pin;           //< Pin number (0-9)
    }; // 12 bytes

    void _sbp_process();
    void _sbp_process_message();
    bool _attempt_state_update();

    // ************************************************************************
    // Internal Received Messages State
    // ************************************************************************
    uint32_t last_heartbeat_received_ms;
    uint32_t last_injected_data_ms;

    struct sbp_heartbeat_t last_heartbeat;
    struct sbp_gps_time_t  last_gps_time;
    struct sbp_dops_t      last_dops;
    struct sbp_pos_llh_t   last_pos_llh;
    struct sbp_vel_ned_t   last_vel_ned;
    struct sbp_ext_event_t last_event;

    uint32_t               last_full_update_tow;
    uint16_t               last_full_update_wn;

    // ************************************************************************
    // Monitoring and Performance Counting
    // ************************************************************************

    uint32_t crc_error_counter;

    // ************************************************************************
    // Logging to AP_Logger
    // ************************************************************************

    void logging_log_full_update();
    void logging_ext_event();

    int32_t distMod(int32_t tow1_ms, int32_t tow2_ms, int32_t mod);

};
#endif
