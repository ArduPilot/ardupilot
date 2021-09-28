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

//  Femtomes GPS driver for ArduPilot.
//  Code by Rui Zheng <ruizheng@femtomes.com>

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include "AP_GPS_BinaryCRCMessageParser.h"


class AP_GPS_FEMTO : public AP_GPS_Backend, public AP_GPS_BinaryCRCMessageParser
{
public:
    AP_GPS_FEMTO(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    const char *name() const override { return "FEMTO"; }

private:
    virtual uint16_t get_message_body_length_from_header_buff() override{return _femto_msg.header.msg_header.messagelength;};
    virtual uint8_t get_message_header_length_from_header_buff() override{return _femto_msg.header.msg_header.headerlength;};
    virtual uint8_t *get_message_header_buff() override{return (uint8_t *)&_femto_msg.header;};
    virtual uint8_t *get_message_body_buff() override{return (uint8_t *)&_femto_msg.body;};
    virtual bool process_message() override;

    /**
    * femto_msg_header_t is femto data header
    */
    struct PACKED femto_msg_header_t 
    {
        uint8_t 	preamble[3];	    /**< Frame header preamble 0xaa 0x44 0x12 */
        uint8_t 	headerlength;	    /**< Frame header length ,from the beginning 0xaa */
        uint16_t 	messageid;          /**< Frame message id ,example the FEMTO_MSG_ID_UAVGPS 8001*/
        uint8_t 	messagetype;	    /**< Frame message id type */
        uint8_t 	portaddr;	        /**< Frame message port address */
        uint16_t 	messagelength;      /**< Frame message data length,from the beginning headerlength+1,end headerlength + messagelength*/
        uint16_t 	sequence;
        uint8_t 	idletime;	        /**< Frame message idle module time */
        uint8_t 	timestatus;
        uint16_t 	week;
        uint32_t 	tow;
        uint32_t 	recvstatus;
        uint16_t 	resv;
        uint16_t 	recvswver;
    };

    /**
    * femto_uav_gps_t struct need to be packed
    */
    struct PACKED femto_uav_gps_t
    {
        uint64_t 	time_utc_usec;		/** Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0*/
        int32_t 	lat;			    /** Latitude in 1E-7 degrees*/
        int32_t 	lon;			    /** Longitude in 1E-7 degrees*/
        int32_t 	alt;			    /** Altitude in 1E-3 meters above MSL, (millimetres)*/
        int32_t 	alt_ellipsoid;		/** Altitude in 1E-3 meters bove Ellipsoid, (millimetres)*/
        float 		s_variance_m_s;		/** GPS speed accuracy estimate, (metres/sec)*/
        float 		c_variance_rad;		/** GPS course accuracy estimate, (radians)*/
        float 		eph;			    /** GPS horizontal position accuracy (metres)*/
        float 		epv;			    /** GPS vertical position accuracy (metres)*/
        float 		hdop;			    /** Horizontal dilution of precision*/
        float 		vdop;			    /** Vertical dilution of precision*/
        int32_t 	noise_per_ms;		/** GPS noise per millisecond*/
        int32_t 	jamming_indicator;	/** indicates jamming is occurring*/
        float 		vel_m_s;		    /** GPS ground speed, (metres/sec)*/
        float 		vel_n_m_s;		    /** GPS North velocity, (metres/sec)*/
        float 		vel_e_m_s;		    /** GPS East velocity, (metres/sec)*/
        float 		vel_d_m_s;		    /** GPS Down velocity, (metres/sec)*/
        float 		cog_rad;		    /** Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)*/
        int32_t 	timestamp_time_relative;/** timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)*/
        float 		heading;		    /** heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])*/
        uint8_t 	fix_type;		    /** 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
        bool 		vel_ned_valid;		/** True if NED velocity is valid*/
        uint8_t 	satellites_used;	/** Number of satellites used*/
        uint8_t		heading_type;		/**< 0 invalid,5 for float,6 for fix*/
    };

    /** uav status data */
    struct PACKED femto_uav_status_t
    {
        int32_t     master_ant_status;
        int32_t     slave_ant_status;
        uint16_t    master_ant_power;
        uint16_t    slave_ant_power;
        uint32_t    jam_status;
        uint32_t    spoofing_status;
        uint16_t    reserved16_1;
        uint16_t    diff_age;		    /**< in unit of second*/
        uint32_t    base_id;
        uint32_t    reserved32_1;
        uint32_t    reserved32_2;
        uint32_t    sat_number;
        struct PACKED femto_uav_sat_status_data_t{  
            uint8_t   svid;
            uint8_t   system_id;
            uint8_t   cn0;		        /**< in unit of dB-Hz*/
            uint8_t   ele;		        /**< in unit of degree*/
            uint16_t  azi;		        /**< in unit of degree*/
            uint16_t  status;
        } sat_status[64];               /**< uav status of all satellites */
    };



    struct PACKED femto_msg_parser_t
    {
        union PACKED {
            DEFINE_BYTE_ARRAY_METHODS
            femto_uav_status_t uav_status;
            femto_uav_gps_t uav_gps;
        } body;
        uint32_t crc;
        union PACKED {
            DEFINE_BYTE_ARRAY_METHODS
            femto_msg_header_t msg_header;
        } header;
        uint16_t read;
    };

    static const uint8_t FEMTO_PREAMBLE1 = 0xaa;
    static const uint8_t FEMTO_PREAMBLE2 = 0x44;
    static const uint8_t FEMTO_PREAMBLE3 = 0x12;

    static const uint16_t FEMTO_MSG_ID_UAVGPS = 8001;
    static const uint16_t FEMTO_MSG_ID_UAVSTATUS = 8017;

    static const char* const _initialisation_blob[];

    femto_msg_parser_t _femto_msg;

    uint8_t _init_blob_index;
    uint32_t _init_blob_time;

    bool _new_uavstatus;    /**< have new uav status */
    uint32_t _last_uav_status_time;

    bool _new_uavgps;       /**< have new uav gps */
    uint32_t _last_uav_gps_time;
};
