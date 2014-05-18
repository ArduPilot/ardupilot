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

    // Methods
    bool read();

    static bool _detect(struct SBP_detect_state &state, uint8_t data);

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
    static const uint16_t SBP_GPS_TIME_MSGTYPE      = 0x0100;
    static const uint16_t SBP_DOPS_MSGTYPE          = 0x0206;
    static const uint16_t SBP_POS_ECEF_MSGTYPE      = 0x0200;
    static const uint16_t SBP_POS_LLH_MSGTYPE       = 0x0201;
    static const uint16_t SBP_BASELINE_ECEF_MSGTYPE = 0x0202;
    static const uint16_t SBP_BASELINE_NED_MSGTYPE  = 0x0203;
    static const uint16_t SBP_VEL_ECEF_MSGTYPE      = 0x0204;
    static const uint16_t SBP_VEL_NED_MSGTYPE       = 0x0205;

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


    // ************************************************************************
    // Swift Navigation SBP protocol parsing and processing
    // ************************************************************************

    //Pulls data from the port, dispatches messages to processing functions
    void sbp_process();

    //Processes individual messages
    //When a message is received, it sets a sticky bit that it has updated
    //itself. This is used to track when a full update of GPS_State has occurred
    void sbp_process_gpstime(uint8_t* msg);
    void sbp_process_dops(uint8_t* msg);
    void sbp_process_pos_ecef(uint8_t* msg);
    void sbp_process_pos_llh(uint8_t* msg);
    void sbp_process_baseline_ecef(uint8_t* msg);
    void sbp_process_baseline_ned(uint8_t* msg);
    void sbp_process_vel_ecef(uint8_t* msg);
    void sbp_process_vel_ned(uint8_t* msg);

    //Sticky bits to track updating of state
    bool has_updated_pos:1;
    bool has_updated_vel:1;


    // ************************************************************************
    // Monitoring and Performance Counting
    // ************************************************************************

    uint8_t  pos_msg_counter;
    uint8_t  vel_msg_counter;
    uint8_t  dops_msg_counter;
    uint8_t  baseline_msg_counter;
    uint16_t crc_error_counter;
    uint32_t last_healthcheck_millis;

    // ************************************************************************
    // Logging to DataFlash
    // ************************************************************************

    // have we written the logging headers to DataFlash?
    static bool logging_started;

    void logging_write_headers();
    void logging_log_health(float pos_msg_hz, float vel_msg_hz, float dops_msg_hz, float baseline_msg_hz, float crc_error_hz);
    void logging_log_baseline(struct sbp_baseline_ecef_t*);      
};

#endif // __AP_GPS_SBP_H__
