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
//  Septentrio GPS driver for ArduPilot.
//	Code by Michael Oborne
//
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#define SBF_SETUP_MSG "\nsso, Stream1, COM1, PVTGeodetic+DOP+ExtEventPVTGeodetic, msec100\n"

class AP_GPS_SBF : public AP_GPS_Backend
{
public:
    AP_GPS_SBF(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D_RTK; }

    // Methods
    bool read();

    void inject_data(uint8_t *data, uint8_t len);

private:

    bool parse(uint8_t temp);
    bool process_message();

    static const uint8_t SBF_PREAMBLE1 = '$';
    static const uint8_t SBF_PREAMBLE2 = '@';

    uint8_t _init_blob_index = 0;
    uint32_t _init_blob_time = 0;
    const char* _initialisation_blob[5] = {
    "sso, Stream1, COM1, PVTGeodetic+DOP+ExtEventPVTGeodetic, msec100\n",
    "srd, Moderate, UAV\n",
    "sem, PVT, 5\n",
    "spm, Rover, StandAlone+SBAS+DGPS+RTK\n",
    "sso, Stream2, Dsk1, postprocess+event, msec100\n"};
   
    uint32_t last_hdop = 999;
    uint32_t crc_error_counter = 0;
    uint32_t last_injected_data_ms = 0;
    bool validcommand = false;

    struct PACKED msg4007
    {
         uint32_t TOW;
         uint16_t WNc;
         uint8_t Mode;
         uint8_t Error;
         double Latitude;
         double Longitude;
         double Height;
         float Undulation;
         float Vn;
         float Ve;
         float Vu;
         float COG;
         double RxClkBias;
         float RxClkDrift;
         uint8_t TimeSystem;
         uint8_t Datum;
         uint8_t NrSV;
         uint8_t WACorrInfo;
         uint16_t ReferenceID;
         uint16_t MeanCorrAge;
         uint32_t SignalInfo;
         uint8_t AlertFlag;
         // rev1
         uint8_t NrBases;
         uint16_t PPPInfo;
         // rev2
         uint16_t Latency;
         uint16_t HAccuracy;
         uint16_t VAccuracy;
         uint8_t Misc;
    };
  
    struct PACKED msg4001
    {
         uint32_t TOW;
         uint16_t WNc;
         uint8_t NrSV;
         uint8_t Reserved;
         uint16_t PDOP;
         uint16_t TDOP;
         uint16_t HDOP;
         uint16_t VDOP;
         float HPL;
         float VPL;
    };

    union PACKED msgbuffer {
        msg4007 msg4007u;
        msg4001 msg4001u;
        uint8_t bytes[128];
    };

    struct sbf_msg_parser_t
    {
        enum
        {
            PREAMBLE1 = 0,
            PREAMBLE2,
            CRC1,
            CRC2,
            BLOCKID1,
            BLOCKID2,
            LENGTH1,
            LENGTH2,
            DATA
        } sbf_state;
        uint16_t preamble;
        uint16_t crc;
        uint16_t blockid;
        uint16_t length;
        msgbuffer data;
        uint16_t read;
    } sbf_msg;

    void log_ExtEventPVTGeodetic(const msg4007 &temp);
};
