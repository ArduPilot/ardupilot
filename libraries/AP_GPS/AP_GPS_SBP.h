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
//  Swift Navigation SBP GPS driver for ArduPilot.
//	Code by Niels Joubert
//
//  Swift Binary Protocol format: http://docs.swift-nav.com/libswiftnav

#ifndef __AP_GPS_SBP_H__
#define __AP_GPS_SBP_H__

#if GPS_RTK_AVAILABLE

#include <AP_GPS.h>


//OVERALL DESIGN NOTES.
//  Niels Joubert, April 2014
//
//
// REQUIREMENTS:
// 1) We need to update the entire state structure "atomically",
//    which is indicated by returning "true" from read().
//
//      - We use sticky bits to track when each part is updated
//        and return true when all the sticky bits are set
//
// 2) We want to minimize memory usage in the detection routine
//
//      - We use a stripped-down version of the sbp_parser_state_t struct
//        that does not bother to decode the message, it just tracks the CRC.
//

class AP_GPS_SBP : public AP_GPS_Backend
{
public:
	AP_GPS_SBP(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool can_calculate_base_pos(void);

    void calculate_base_pos(void);

    void invalidate_base_pos(void);

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    bool read();

    static bool _detect(struct SBP_detect_state &state, uint8_t data);

    virtual void send_mavlink_gps_rtk(mavlink_channel_t chan);

#if GPS_MAX_INSTANCES > 1
    virtual void send_mavlink_gps2_rtk(mavlink_channel_t chan);
#endif

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
    
    //Message types supported by this driver
    static const uint16_t SBP_STARTUP_MSGTYPE        = 0xFF00;    
    static const uint16_t SBP_HEARTBEAT_MSGTYPE      = 0xFFFF;    
    static const uint16_t SBP_GPS_TIME_MSGTYPE       = 0x0100;
    static const uint16_t SBP_DOPS_MSGTYPE           = 0x0206;
    static const uint16_t SBP_POS_ECEF_MSGTYPE       = 0x0200;
    static const uint16_t SBP_POS_LLH_MSGTYPE        = 0x0201;
    static const uint16_t SBP_BASELINE_ECEF_MSGTYPE  = 0x0202;
    static const uint16_t SBP_BASELINE_NED_MSGTYPE   = 0x0203;
    static const uint16_t SBP_VEL_ECEF_MSGTYPE       = 0x0204;
    static const uint16_t SBP_VEL_NED_MSGTYPE        = 0x0205;
    static const uint16_t SBP_TRACKING_STATE_MSGTYPE = 0x0016;
    static const uint16_t SBP_IAR_STATE_MSGTYPE      = 0x0019;
    
    // GPS Time
    struct PACKED sbp_gps_time_t {
        uint16_t wn;     //< GPS week number (unit: weeks)
        uint32_t tow;    //< GPS Time of Week rounded to the nearest ms (unit: ms)
        int32_t ns;      //< Nanosecond remainder of rounded tow (unit: ns)
        uint8_t flags;   //< Status flags (reserved)
    };

    // Dilution of Precision
    struct PACKED sbp_dops_t {
        uint32_t tow;    //< GPS Time of Week (unit: ms)
        uint16_t gdop;   //< Geometric Dilution of Precision (unit: 0.01)
        uint16_t pdop;   //< Position Dilution of Precision (unit: 0.01)
        uint16_t tdop;   //< Time Dilution of Precision (unit: 0.01)
        uint16_t hdop;   //< Horizontal Dilution of Precision (unit: 0.01)
        uint16_t vdop;   //< Vertical Dilution of Precision (unit: 0.01)
    };

    // Position solution in absolute Earth Centered Earth Fixed (ECEF) coordinates.
    struct PACKED sbp_pos_ecef_t {
        uint32_t tow;       //< GPS Time of Week (unit: ms)
        double x;           //< ECEF X coordinate (unit: meters)
        double y;           //< ECEF Y coordinate (unit: meters)
        double z;           //< ECEF Z coordinate (unit: meters)
        uint16_t accuracy;  //< Position accuracy estimate (unit: mm)
        uint8_t n_sats;     //< Number of satellites used in solution
        uint8_t flags;      //< Status flags
    };

    // Geodetic position solution.
    struct PACKED sbp_pos_llh_t {
        uint32_t tow;         //< GPS Time of Week (unit: ms)
        double lat;           //< Latitude (unit: degrees)
        double lon;           //< Longitude (unit: degrees)
        double height;        //< Height (unit: meters)
        uint16_t h_accuracy;  //< Horizontal position accuracy estimate (unit: mm)
        uint16_t v_accuracy;  //< Vertical position accuracy estimate (unit: mm)
        uint8_t n_sats;       //< Number of satellites used in solution
        uint8_t flags;        //< Status flags
    };

    // Baseline in Earth Centered Earth Fixed (ECEF) coordinates.
    struct PACKED sbp_baseline_ecef_t {
        uint32_t tow;        //< GPS Time of Week (unit: ms)
        int32_t x;           //< Baseline ECEF X coordinate (unit: mm)
        int32_t y;           //< Baseline ECEF Y coordinate (unit: mm)
        int32_t z;           //< Baseline ECEF Z coordinate (unit: mm)
        uint16_t accuracy;   //< Position accuracy estimate (unit: mm)
        uint8_t n_sats;      //< Number of satellites used in solution
        uint8_t flags;       //< Status flags (reserved)
    };

    // Baseline in local North East Down (NED) coordinates.
    struct PACKED sbp_baseline_ned_t {
        uint32_t tow;          //< GPS Time of Week (unit: ms)
        int32_t n;             //< Baseline North coordinate (unit: mm)
        int32_t e;             //< Baseline East coordinate  (unit: mm)
        int32_t d;             //< Baseline Down coordinate  (unit: mm)
        uint16_t h_accuracy;   //< Horizontal position accuracy estimate (unit: mm)
        uint16_t v_accuracy;   //< Vertical position accuracy estimate (unit: mm)
        uint8_t n_sats;        //< Number of satellites used in solution
        uint8_t flags;         //< Status flags (reserved)
    };

    //Velocity in Earth Centered Earth Fixed (ECEF) coordinates.
    struct PACKED sbp_vel_ecef_t {
        uint32_t tow;          //< GPS Time of Week (unit: ms)
        int32_t x;             //< Velocity ECEF X coordinate (unit: mm/s)
        int32_t y;             //< Velocity ECEF Y coordinate (unit: mm/s)
        int32_t z;             //< Velocity ECEF Z coordinate (unit: mm/s)
        uint16_t accuracy;     //< Velocity accuracy estimate (unit: mm/s)
        uint8_t n_sats;        //< Number of satellites used in solution
        uint8_t flags;         //< Status flags (reserved)
    };

    // Velocity in NED Velocity in local North East Down (NED) coordinates.
    struct PACKED sbp_vel_ned_t {
        uint32_t tow;          //< GPS Time of Week (unit: ms)
        int32_t n;             //< Velocity North coordinate (unit: mm/s)
        int32_t e;             //< Velocity East coordinate  (unit: mm/s)
        int32_t d;             //< Velocity Down coordinate  (unit: mm/s)
        uint16_t h_accuracy;   //< Horizontal velocity accuracy estimate (unit: mm/s)
        uint16_t v_accuracy;   //< Vertical velocity accuracy estimate (unit: mm/s)
        uint8_t n_sats;        //< Number of satellites used in solution 
        uint8_t flags;         //< Status flags (reserved)
    };

    // Activity and Signal-to-Noise data of a tracking channel on the GPS.
    struct PACKED sbp_tracking_state_t {
        uint8_t state;         //< 0 if disabled, 1 if running
        uint8_t prn;           //< PRN identifier of tracked satellite
        float cn0;             //< carrier to noise power ratio.
    };

    // Integer Ambiguity Resolution state - how is the RTK resolution doing?
    struct PACKED sbp_iar_state_t {
        uint32_t num_hypotheses;
    };



    // ************************************************************************
    // Swift Navigation SBP protocol parsing and processing
    // ************************************************************************

    //Pulls data from the port, dispatches messages to processing functions
    //Returns true if a new message was successfully decoded.
    bool sbp_process();

    bool update_state(bool has_new_message);
    
    void update_state_velocity(void);

    //Processes individual messages
    //When a message is received, it sets a sticky bit that it has updated
    //itself. This is used to track when a full update of GPS_State has occurred
    void sbp_process_heartbeat(uint8_t* msg);    
    void sbp_process_gpstime(uint8_t* msg);
    void sbp_process_dops(uint8_t* msg);
    void sbp_process_pos_ecef(uint8_t* msg);
    void sbp_process_pos_llh(uint8_t* msg);
    void sbp_process_baseline_ecef(uint8_t* msg);
    void sbp_process_baseline_ned(uint8_t* msg);
    void sbp_process_vel_ecef(uint8_t* msg);
    void sbp_process_vel_ned(uint8_t* msg);
    void sbp_process_tracking_state(uint8_t* msg, uint8_t len);
    void sbp_process_iar_state(uint8_t* msg);
    void sbp_process_startup(uint8_t* msg);


    //Internal last-received-messages
    sbp_pos_llh_t            last_sbp_pos_llh_msg;
    sbp_vel_ned_t            last_sbp_vel_ned_msg;
    sbp_baseline_ecef_t      last_sbp_baseline_ecef_msg;
    sbp_baseline_ned_t       last_sbp_baseline_ned_msg;
    sbp_tracking_state_t     last_sbp_tracking_state_msg;
    uint8_t                  last_sbp_tracking_state_msg_num;

    //Tracking GPS health and received time-of-week
    uint32_t last_baseline_received_ms;
    uint32_t last_heatbeat_received_ms;
    uint32_t last_tracking_state_ms;
    int32_t iar_num_hypotheses;
    uint8_t baseline_recv_rate; //in hertz * 10

    //Sticky bits to track updating of state
    bool dgps_corrections_incoming:1;
    bool rtk_corrections_incoming:1;

    bool has_new_pos_llh:1;
    bool has_new_vel_ned:1;
    bool has_new_baseline_ecef:1;

    //RTK-specific relative-to-absolute positioning
    bool has_rtk_base_pos:1;
    Vector3d base_pos_ecef;
    
    // ************************************************************************
    // Monitoring and Performance Counting
    // ************************************************************************

    uint8_t  pos_msg_counter;
    uint8_t  vel_msg_counter;
    uint8_t  baseline_msg_counter;
    uint8_t  full_update_counter;
    uint32_t crc_error_counter;
    uint32_t last_healthcheck_millis;

    // ************************************************************************
    // Logging to DataFlash
    // ************************************************************************

    // have we written the logging headers to DataFlash?
    static bool logging_started;

    void logging_write_headers();

    void logging_log_health(float pos_msg_hz, float vel_msg_hz, float baseline_msg_hz, float full_update_hz);
    void logging_log_llh(struct sbp_pos_llh_t* p);
    void logging_log_baseline_ecef(struct sbp_baseline_ecef_t*);      
    void logging_log_tracking_state(struct sbp_tracking_state_t*, uint8_t num);      

};

#endif //  GPS_RTK_AVAILABLE

#endif // __AP_GPS_SBP_H__ 
