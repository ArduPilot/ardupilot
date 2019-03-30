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
#define SBF_DISK_ACTIVITY (1 << 7)
#define SBF_DISK_FULL     (1 << 8)
#define SBF_DISK_MOUNTED  (1 << 9)

class AP_GPS_SBF : public AP_GPS_Backend
{
public:
    AP_GPS_SBF(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    const char *name() const override { return "SBF"; }

    bool is_configured (void) override;

    void broadcast_configuration_failure_reason(void) const override;

    // get the velocity lag, returns true if the driver is confident in the returned value
    bool get_lag(float &lag_sec) const override { lag_sec = 0.08f; return true; } ;

    bool is_healthy(void) const override;

    bool prepare_for_arming(void) override;


private:

    bool parse(uint8_t temp);
    bool process_message();

    static const uint8_t SBF_PREAMBLE1 = '$';
    static const uint8_t SBF_PREAMBLE2 = '@';

    uint8_t _init_blob_index = 0;
    uint32_t _init_blob_time = 0;
    const char* _initialisation_blob[5] = {
    "sso, Stream1, COM1, PVTGeodetic+DOP+ExtEventPVTGeodetic+ReceiverStatus+VelCovGeodetic, msec100\n",
    "srd, Moderate, UAV\n",
    "sem, PVT, 5\n",
    "spm, Rover, all\n",
    "sso, Stream2, Dsk1, postprocess+event+comment, msec100\n"};
    uint32_t _config_last_ack_time;

    const char* _port_enable = "\nSSSSSSSSSS\n";
   
    uint32_t crc_error_counter = 0;
    uint32_t RxState;
    uint32_t RxError;

    void mount_disk(void) const;
    void unmount_disk(void) const;
    bool _has_been_armed;

    enum sbf_ids {
        DOP = 4001,
        PVTGeodetic = 4007,
        ReceiverStatus = 4014,
        ExtEventPVTGeodetic = 4038,
        VelCovGeodetic = 5908
    };

    struct PACKED msg4007 // PVTGeodetic
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
  
    struct PACKED msg4001 // DOP
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

    struct PACKED msg4014 // ReceiverStatus (v2)
    {
         uint32_t TOW;
         uint16_t WNc;
         uint8_t CPULoad;
         uint8_t ExtError;
         uint32_t UpTime;
         uint32_t RxState;
         uint32_t RxError;
         // remaining data is AGCData, which we don't have a use for, don't extract the data
    };

    struct PACKED msg5908 // VelCovGeodetic
    {
        uint32_t TOW;
        uint16_t WNc;
        uint8_t Mode;
        uint8_t Error;
        float Cov_VnVn;
        float Cov_VeVe;
        float Cov_VuVu;
        float Cov_DtDt;
        float Cov_VnVe;
        float Cov_VnVu;
        float Cov_VnDt;
        float Cov_VeVu;
        float Cov_VeDt;
        float Cov_VuDt;
    };

    union PACKED msgbuffer {
        msg4007 msg4007u;
        msg4001 msg4001u;
        msg4014 msg4014u;
        msg5908 msg5908u;
        uint8_t bytes[256];
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
            DATA,
            COMMAND_LINE // used to parse command responses
        } sbf_state;
        uint16_t preamble;
        uint16_t crc;
        uint16_t blockid;
        uint16_t length;
        msgbuffer data;
        uint16_t read;
    } sbf_msg;

    void log_ExtEventPVTGeodetic(const msg4007 &temp);

    enum {
        SOFTWARE      = (1 << 3),   // set upon detection of a software warning or  error. This bit is reset by the command  “lif, error”
        WATCHDOG      = (1 << 4),   // set when the watch-dog expired at least once since the last power-on.
        CONGESTION    = (1 << 6),   // set when an output data congestion has been detected on at least one of the communication ports of the receiver during the last second.
        MISSEDEVENT   = (1 << 8),   // set when an external event congestion has been detected during the last second. It indicates that the receiver is receiving too many events on its EVENTx pins.
        CPUOVERLOAD   = (1 << 9),   // set when the CPU load is larger than 90%. If this bit is set, receiver operation may be unreliable and the user must decrease the processing load by following the recommendations in the User Manual.
        INVALIDCONFIG = (1 << 10),  // set if one or more configuration file (permission or channel configuration) is invalid or absent.
        OUTOFGEOFENCE = (1 << 11),  // set if the receiver is currently out of its permitted region of operation (geo-fencing).
    };
};
