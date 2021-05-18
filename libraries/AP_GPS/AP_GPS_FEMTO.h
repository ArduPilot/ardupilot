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

class AP_GPS_FEMTO : public AP_GPS_Backend
{
public:
    AP_GPS_FEMTO(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    void inject_data(const uint8_t *data, uint16_t len) override;

    const char *name() const override { return "FEMTO"; }

private:
    bool parse(uint8_t temp);
    bool process_message();
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);

    static const uint8_t FEMTO_PREAMBLE1 = 0xaa;
    static const uint8_t FEMTO_PREAMBLE2 = 0x44;
    static const uint8_t FEMTO_PREAMBLE3 = 0x12;

    static const uint16_t FEMTO_MSG_ID_UAVGPS = 8001;
    static const uint8_t  FEMTO_MSG_ID_BESTPOS = 42;
    
    static const char* const _initialisation_blob[3];
   
    uint32_t crc_error_counter;
    uint32_t last_injected_data_ms;

    uint8_t _init_blob_index;
    uint32_t _init_blob_time;

    // do we have new position information?
    bool _new_bestpos;
    // do we have new uavgps information?
    bool _new_uavgps;
    
    uint32_t _last_vel_time;

    /**
    * femto_msg_header_t is femto data header
    */
    struct PACKED femto_msg_header_t 
    {
        uint8_t 	preamble[3];	/**< Frame header preamble 0xaa 0x44 0x12 */
        uint8_t 	headerlength;	/**< Frame header length ,from the beginning 0xaa */
        uint16_t 	messageid;     /**< Frame message id ,example the FEMTO_MSG_ID_UAVGPS 8001*/
        uint8_t 	messagetype;	/**< Frame message id type */
        uint8_t 	portaddr;	/**< Frame message port address */
        uint16_t 	messagelength; /**< Frame message data length,from the beginning headerlength+1,end headerlength + messagelength*/
        uint16_t 	sequence;
        uint8_t 	idletime;	/**< Frame message idle module time */
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
        int32_t 	lat;			/** Latitude in 1E-7 degrees*/
        int32_t 	lon;			/** Longitude in 1E-7 degrees*/
        int32_t 	alt;			/** Altitude in 1E-3 meters above MSL, (millimetres)*/
        int32_t 	alt_ellipsoid;		/** Altitude in 1E-3 meters bove Ellipsoid, (millimetres)*/
        float 		s_variance_m_s;		/** GPS speed accuracy estimate, (metres/sec)*/
        float 		c_variance_rad;		/** GPS course accuracy estimate, (radians)*/
        float 		eph;			/** GPS horizontal position accuracy (metres)*/
        float 		epv;			/** GPS vertical position accuracy (metres)*/
        float 		hdop;			/** Horizontal dilution of precision*/
        float 		vdop;			/** Vertical dilution of precision*/
        int32_t 	noise_per_ms;		/** GPS noise per millisecond*/
        int32_t 	jamming_indicator;	/** indicates jamming is occurring*/
        float 		vel_m_s;		/** GPS ground speed, (metres/sec)*/
        float 		vel_n_m_s;		/** GPS North velocity, (metres/sec)*/
        float 		vel_e_m_s;		/** GPS East velocity, (metres/sec)*/
        float 		vel_d_m_s;		/** GPS Down velocity, (metres/sec)*/
        float 		cog_rad;		/** Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)*/
        int32_t 	timestamp_time_relative;/** timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)*/
        float 		heading;		/** heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])*/
        uint8_t 	fix_type;		/** 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
        bool 		vel_ned_valid;		/** True if NED velocity is valid*/
        uint8_t 	satellites_used;	/** Number of satellites used*/
        uint8_t		heading_type;		/**< 0 invalid,5 for float,6 for fix*/
    };

    struct PACKED femto_best_pos_t
    {
        uint32_t solstat;      ///< Solution status
        uint32_t postype;      ///< Position type
        double lat;            ///< latitude (deg)
        double lng;            ///< longitude (deg)
        double hgt;            ///< height above mean sea level (m)
        float undulation;      ///< relationship between the geoid and the ellipsoid (m)
        uint32_t datumid;      ///< datum id number
        float latsdev;         ///< latitude standard deviation (m)
        float lngsdev;         ///< longitude standard deviation (m)
        float hgtsdev;         ///< height standard deviation (m)
        // 4 bytes
        uint8_t stnid[4];      ///< base station id
        float diffage;         ///< differential position age (sec)
        float sol_age;         ///< solution age (sec)
        uint8_t svstracked;    ///< number of satellites tracked
        uint8_t svsused;       ///< number of satellites used in solution
        uint8_t svsl1;         ///< number of GPS plus GLONASS L1 satellites used in solution
        uint8_t svsmultfreq;   ///< number of GPS plus GLONASS L2 satellites used in solution
        uint8_t resv;          ///< reserved
        uint8_t extsolstat;    ///< extended solution status - OEMV and greater only
        uint8_t galbeisigmask;
        uint8_t gpsglosigmask;
    };
    
    union PACKED msg_buffer_t {
        femto_best_pos_t best_pos;
        femto_uav_gps_t uav_gps;
        uint8_t bytes[256];
    };
    
    union PACKED msg_header_t {
        femto_msg_header_t femto_header;
        uint8_t data[28];
    };

    struct PACKED femto_msg_parser_t
    {
        enum
        {
            PREAMBLE1 = 0,
            PREAMBLE2,
            PREAMBLE3,
            HEADERLENGTH,
            HEADERDATA,
            DATA,
            CRC1,
            CRC2,
            CRC3,
            CRC4,
        } femto_decode_state;
        
        msg_buffer_t data;
        uint32_t crc;
        msg_header_t header;
        uint16_t read;
    } femto_msg;
};
