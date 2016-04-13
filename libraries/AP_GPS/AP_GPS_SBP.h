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
//  Swift Binary Protocol format: http://docs.swift-nav.com/
//
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

class AP_GPS_SBP : public AP_GPS_Backend
{
public:
    AP_GPS_SBP(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    // Methods
    bool read();

    void inject_data(uint8_t *data, uint8_t len);

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
    }; // 11 bytes

    // Dilution of Precision
    struct PACKED sbp_dops_t {
        uint32_t tow;    //< GPS Time of Week (unit: ms)
        uint16_t gdop;   //< Geometric Dilution of Precision (unit: 0.01)
        uint16_t pdop;   //< Position Dilution of Precision (unit: 0.01)
        uint16_t tdop;   //< Time Dilution of Precision (unit: 0.01)
        uint16_t hdop;   //< Horizontal Dilution of Precision (unit: 0.01)
        uint16_t vdop;   //< Vertical Dilution of Precision (unit: 0.01)
    }; // 14 bytes

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
        uint8_t flags;         //< Status flags (reserved)
    }; // 22 bytes

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

    void _sbp_process();
    void _sbp_process_message();
    bool _attempt_state_update();

    // ************************************************************************
    // Internal Received Messages State
    // ************************************************************************
    uint32_t last_heatbeat_received_ms;
    uint32_t last_injected_data_ms;

    struct sbp_gps_time_t last_gps_time;
    struct sbp_dops_t     last_dops;
    struct sbp_pos_llh_t  last_pos_llh_spp;
    struct sbp_pos_llh_t  last_pos_llh_rtk;
    struct sbp_vel_ned_t  last_vel_ned;
    uint32_t              last_iar_num_hypotheses;

    uint32_t              last_full_update_tow;
    uint32_t              last_full_update_cpu_ms;

    // ************************************************************************
    // Monitoring and Performance Counting
    // ************************************************************************

    uint32_t crc_error_counter;

    // ************************************************************************
    // Logging to DataFlash
    // ************************************************************************

    void logging_log_full_update();
    void logging_log_raw_sbp(uint16_t msg_type, uint16_t sender_id, uint8_t msg_len, uint8_t *msg_buff);
   

};
