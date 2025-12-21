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

//  Novatel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by Michael Oborne
//  Derived from https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Manuals/om-20000129/om-20000129.pdf

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_NOVA_ENABLED
class AP_GPS_NOVA : public AP_GPS_Backend
{
public:
    AP_GPS_NOVA(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    const char *name() const override { return "NOVA"; }

private:

    bool parse(uint8_t temp);
    bool process_message();

    static const uint8_t NOVA_PREAMBLE1 = 0xaa;
    static const uint8_t NOVA_PREAMBLE2 = 0x44;
    static const uint8_t NOVA_PREAMBLE3 = 0x12;

    // do we have new position information?
    bool            _new_position:1;
    // do we have new speed information?
    bool            _new_speed:1;
    
    uint32_t        _last_vel_time;
    
    uint8_t _init_blob_index = 0;
    uint32_t _init_blob_time = 0;
    static const char* const _initialisation_blob[4];
   
    uint32_t crc_error_counter = 0;

    struct PACKED nova_header
    {
        // 0
        uint8_t preamble[3];
        // 3
        uint8_t headerlength;
        // 4
        uint16_t messageid;
        // 6
        uint8_t messagetype;
        //7
        uint8_t portaddr;
        //8
        uint16_t messagelength;
        //10
        uint16_t sequence;
        //12
        uint8_t idletime;
        //13
        uint8_t timestatus;
        //14
        uint16_t week;
        //16
        uint32_t tow;
        //20
        uint32_t recvstatus;
        // 24
        uint16_t resv;
        //26
        uint16_t recvswver;
    };    

    static const uint8_t NOVA_PSRDOP = 174;
    struct PACKED psrdop
    {
        float gdop;
        float pdop;
        float hdop;
        float htdop;
        float tdop;
        float cutoff;
        uint32_t svcount;
        // extra data for individual prns
    };

    static const uint8_t NOVA_BESTPOS = 42;
    struct PACKED bestpos
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

    static const uint8_t NOVA_BESTVEL = 99;
    struct PACKED bestvel
    {
        uint32_t solstat;
        uint32_t veltype;
        float latency;
        float age;
        double horspd;
        double trkgnd;
        // + up
        double vertspd;
        float resv;
    };
    
    union PACKED msgbuffer {
        bestvel bestvelu;
        bestpos bestposu;
        psrdop psrdopu;
        uint8_t bytes[256];
    };
    
    union PACKED msgheader {
        nova_header nova_headeru;
        uint8_t data[28];
    };

    struct PACKED nova_msg_parser
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
        } nova_state;
        
        msgbuffer data;
        uint32_t crc;
        msgheader header;
        uint16_t read;
    } nova_msg;
};
#endif
