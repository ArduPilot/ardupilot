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

#if AP_GPS_SBF_ENABLED

#define SBF_DISK_ACTIVITY (1 << 7)
#define SBF_DISK_FULL     (1 << 8)
#define SBF_DISK_MOUNTED  (1 << 9)

class AP_GPS_SBF : public AP_GPS_Backend
{
public:
    AP_GPS_SBF(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);
    ~AP_GPS_SBF();

    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    // Methods
    bool read() override;

    const char *name() const override { return "SBF"; }

    bool is_configured (void) const override;

    void broadcast_configuration_failure_reason(void) const override;

#if HAL_GCS_ENABLED
    bool supports_mavlink_gps_rtk_message(void) const override { return true; };
#endif

    // get the velocity lag, returns true if the driver is confident in the returned value
    bool get_lag(float &lag_sec) const override { lag_sec = 0.08f; return true; } ;

    bool is_healthy(void) const override;

    bool logging_healthy(void) const override;

    bool prepare_for_arming(void) override;

    bool get_error_codes(uint32_t &error_codes) const override { error_codes = RxError; return true; };

private:

    bool parse(uint8_t temp);
    bool process_message();

    static const uint8_t SBF_PREAMBLE1 = '$';
    static const uint8_t SBF_PREAMBLE2 = '@';

    uint8_t _init_blob_index;
    uint32_t _init_blob_time;
    enum class Config_State {
        Baud_Rate,
        SSO,
        Blob,
        SBAS,
        SGA,
        Complete
    };
    Config_State config_step;
    char *config_string;
    static constexpr const char* _initialisation_blob[] = {
    "srd,Moderate,UAV",
    "sem,PVT,5",
    "spm,Rover,all",
    "sso,Stream2,Dsk1,postprocess+event+comment+ReceiverStatus,msec100",
#if defined (GPS_SBF_EXTRA_CONFIG)
    GPS_SBF_EXTRA_CONFIG
#endif
    };
    static constexpr const char* sbas_off = "sst, -SBAS";
    static constexpr const char* sbas_on_blob[] = {
                                                   "snt,+GEOL1+GEOL5",
                                                   "sst,+SBAS",
                                                   "ssbc,auto,Operational,MixedSystems,auto",
                                                  };
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
        BaseVectorGeod = 4028,
        VelCovGeodetic = 5908,
        AttEulerCov = 5939,
        AuxAntPositions = 5942,
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

    struct PACKED VectorInfoGeod {
        uint8_t NrSV;
        uint8_t Error;
        uint8_t Mode;
        uint8_t Misc;
        double DeltaEast;
        double DeltaNorth;
        double DeltaUp;
        float DeltaVe;
        float DeltaVn;
        float DeltaVu;
        uint16_t Azimuth;
        int16_t Elevation;
        uint8_t ReferenceID;
        uint16_t CorrAge;
        uint32_t SignalInfo;
    };

    struct PACKED msg4028 // BaseVectorGeod
    {
        uint32_t TOW;
        uint16_t WNc;
        uint8_t N; // number of baselines
        uint8_t SBLength;
        VectorInfoGeod info; // there can be multiple baselines here, but we will only consume the first one, so don't worry about anything after
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

    struct PACKED msg5939       // AttEulerCoV
    {
        uint32_t TOW;           // receiver time stamp, 0.001s
        uint16_t WNc;           // receiver time stamp, 1 week
        uint8_t Reserved;       // unused
        uint8_t Error;          // error code.  bit 0-1:antenna 1, bit 2-3:antenna2, bit 7: when att not requested
                                //   00b:no error, 01b:not enough meausurements, 10b:antennas are on one line, 11b:inconsistent with manual anntena pos info
        float Cov_HeadHead;     // heading estimate variance
        float Cov_PitchPitch;   // pitch estimate variance
        float Cov_RollRoll;     // roll estimate variance
        float Cov_HeadPitch;    // covariance between Euler angle estimates.  Always set to Do-No-Use values
        float Cov_HeadRoll;
        float Cov_PitchRoll;
    };

    struct PACKED AuxAntPositionSubBlock {
        uint8_t NrSV;           // total number of satellites tracked by the antenna
        uint8_t Error;          // aux antenna position error code
        uint8_t AmbiguityType;  // aux antenna positions obtained with 0: fixed ambiguities, 1: float ambiguities
        uint8_t AuxAntID;       // aux antenna ID: 1 for the first auxiliary antenna, 2 for the second, etc.
        double DeltaEast;       // position in East direction (relative to main antenna)
        double DeltaNorth;      // position in North direction (relative to main antenna)
        double DeltaUp;         // position in Up direction (relative to main antenna)
        double EastVel;         // velocity in East direction (relative to main antenna)
        double NorthVel;        // velocity in North direction (relative to main antenna)
        double UpVel;           // velocity in Up direction (relative to main antenna)
    };

    struct PACKED msg5942   // AuxAntPositions
    {
        uint32_t TOW;
        uint16_t WNc;
        uint8_t N;          // number of AuxAntPosition sub-blocks in this AuxAntPositions block
        uint8_t SBLength;   // length of one sub-block in bytes
        AuxAntPositionSubBlock ant1;    // first aux antennas position
    };

    union PACKED msgbuffer {
        msg4007 msg4007u;
        msg4001 msg4001u;
        msg4014 msg4014u;
        msg4028 msg4028u;
        msg5908 msg5908u;
        msg5939 msg5939u;
        msg5942 msg5942u;
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

    enum {
        SOFTWARE      = (1 << 3),   // set upon detection of a software warning or  error. This bit is reset by the command lif, error
        WATCHDOG      = (1 << 4),   // set when the watch-dog expired at least once since the last power-on.
        CONGESTION    = (1 << 6),   // set when an output data congestion has been detected on at least one of the communication ports of the receiver during the last second.
        MISSEDEVENT   = (1 << 8),   // set when an external event congestion has been detected during the last second. It indicates that the receiver is receiving too many events on its EVENTx pins.
        CPUOVERLOAD   = (1 << 9),   // set when the CPU load is larger than 90%. If this bit is set, receiver operation may be unreliable and the user must decrease the processing load by following the recommendations in the User Manual.
        INVALIDCONFIG = (1 << 10),  // set if one or more configuration file (permission or channel configuration) is invalid or absent.
        OUTOFGEOFENCE = (1 << 11),  // set if the receiver is currently out of its permitted region of operation (geo-fencing).
    };

    static constexpr const char *portIdentifiers[] = { "COM", "USB", "IP1", "NTR", "IPS", "IPR" };
    char portIdentifier[5];
    uint8_t portLength;
    bool readyForCommand;
};
#endif
