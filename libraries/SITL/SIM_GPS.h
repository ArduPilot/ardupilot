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
/*
  simulate GPS sensors

  Usage example:
param set SERIAL5_PROTOCOL 5

     sim_vehicle.py -D --console --map -A --uartB=sim:gps:2
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_GPS_ENABLED
#define HAL_SIM_GPS_ENABLED AP_SIM_ENABLED
#endif

#if HAL_SIM_GPS_ENABLED

#ifndef AP_SIM_GPS_FILE_ENABLED
// really need to use AP_FileSystem for this.
#define AP_SIM_GPS_FILE_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#endif

#include "SIM_SerialDevice.h"

namespace SITL {

// for delay simulation:
struct GPS_Data {
    uint32_t timestamp_ms;
    double latitude;
    double longitude;
    float altitude;
    double speedN;
    double speedE;
    double speedD;
    double yaw_deg;
    double roll_deg;
    double pitch_deg;
    bool have_lock;

    // Get heading [rad], where 0 = North in WGS-84 coordinate system
    float heading() const WARN_IF_UNUSED;

    // Get 2D speed [m/s] in WGS-84 coordinate system
    float speed_2d() const WARN_IF_UNUSED;
};


class GPS_Backend {
public:
    CLASS_NO_COPY(GPS_Backend);

    GPS_Backend(class GPS &front, uint8_t _instance);
    virtual ~GPS_Backend() {}

    // 0 baud means "unset" i.e. baud-rate checks should not apply
    virtual uint32_t device_baud() const { return 0; }

    ssize_t write_to_autopilot(const char *p, size_t size) const;
    ssize_t read_from_autopilot(char *buffer, size_t size) const;

    // read and process config from autopilot (e.g.)
    virtual void update_read();
    // writing fix information to autopilot (e.g.)
    virtual void publish(const GPS_Data *d) = 0;
    
protected:

    uint8_t instance;
    GPS &front;

    class SIM *_sitl;

};

class GPS_FILE : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_FILE);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;
};

class DCOL_Parser {
    // The DCOL parser is used by Trimble GSOF devices.
    // It's used for doing configuration.
    // https://receiverhelp.trimble.com/oem-gnss/API_DataCollectorFormatPackets.html
public:
    // Feed data in to the DCOL parser.
    // If the data reaches a parse state that needs to write ACK/NACK back out,
    // the function returns true with a populated data_out value.
    // Otherwise, it returns false waiting for more data.
    bool dcol_parse(const char data_in);

    static constexpr uint8_t STX = 0x02;
    static constexpr uint8_t ETX = 0x03;

    // Receiver status code
    enum class Status : uint8_t {
        OK = 0x00,
    };

    // https://receiverhelp.trimble.com/oem-gnss/API_DataCollectorFormatPackets.html
    enum class Command_Response : uint8_t  {
        ACK = 0x06,
        NACK = 0x15,
    };

    // https://receiverhelp.trimble.com/oem-gnss/ICD_Command64h_AppFile_Output.html#Frequenc
    enum class Output_Rate : uint8_t {
        OFF = 0,
        FREQ_10_HZ = 1,
        FREQ_50_HZ = 15,
        FREQ_100_HZ = 16,
    };

    // https://receiverhelp.trimble.com/oem-gnss/ICD_ApplicationFilePackets.html?tocpath=API%20Documentation%7CCommand%20and%20report%20packets%7CApplication%20file%20packets%7C_____0
    enum class Packet_Type : uint8_t {
        COMMAND_APPFILE = 0x64,
    };

    // https://receiverhelp.trimble.com/oem-gnss/ICD_Pkt_Command64h_APPFILE.html
    enum class Appfile_Record_Type : uint8_t {
        SERIAL_PORT_BAUD_RATE_FORMAT = 0x02,
        OUTPUT_MESSAGE = 0x07,
    };

    // https://receiverhelp.trimble.com/oem-gnss/ICD_Command64h_AppFile_Output.html#Output
    enum class Output_Msg_Msg_Type : uint8_t {
        GSOF = 10,
    };

    // https://receiverhelp.trimble.com/oem-gnss/ICD_Command64h_AppFile_Output.html#Output2
    enum class Gsof_Msg_Record_Type : uint8_t {
        POSITION_TIME = 1,
        LLH = 2,
        VELOCITY_DATA = 8,
        PDOP_INFO = 9,
        POSITION_SIGMA_INFO = 12,
    };

protected:
    // https://receiverhelp.trimble.com/oem-gnss/API_DataCollectorFormatPacketStructure.html
    static constexpr uint8_t MAX_PAYLOAD_SIZE = 255;

    // GSOF supports this many different packet types.
    // Only a fraction are supported by the simulator.
    // Waste some RAM and allocate arrays for the whole set.
    // https://receiverhelp.trimble.com/oem-gnss/ICD_Command64h_AppFile_Output.html#Output2
    static constexpr uint8_t MAX_CHANNEL_NUM = 70;
    // Rates of dynamically enabled channels.
    // Assume factory behavior of no enabled channels.
    // Each channel can send data out at its own rate.
    Output_Rate channel_rates[MAX_CHANNEL_NUM] = {Output_Rate::OFF};

    // Last publish time of dynamically enabled channels.
    uint32_t last_publish_ms[MAX_CHANNEL_NUM];

    static uint32_t RateToPeriodMs(const Output_Rate rate);

private:

    // Internal parser implementation state
    enum class Parse_State {
        WAITING_ON_STX,
        WAITING_ON_STATUS,
        WAITING_ON_PACKET_TYPE,
        WAITING_ON_LENGTH,
        WAITING_ON_PACKET_DATA,
        WAITING_ON_CSUM,
        WAITING_ON_ETX,
    };

    bool valid_csum();
    bool parse_payload();
    // https://receiverhelp.trimble.com/oem-gnss/ICD_Pkt_Command64h_APPFILE.html
    bool parse_cmd_appfile();


    // states for currently parsing packet
    Status status;
    Parse_State parse_state = {Parse_State::WAITING_ON_STX};
    Packet_Type packet_type;
    // This is the length in the header.
    uint8_t expected_payload_length;
    // This is the increasing tally of bytes per packet.
    uint8_t cur_payload_idx;
    // This is the expected packet checksum in the trailer.
    uint8_t expected_csum;

    // The application file record transmission number
    uint8_t appfile_trans_num;

    uint8_t payload[MAX_PAYLOAD_SIZE];

    // Clear all parser state/flags for handling a fresh packet.
    void reset();
};

class GPS_GSOF : public GPS_Backend, public DCOL_Parser {
public:
    CLASS_NO_COPY(GPS_GSOF);

    using GPS_Backend::GPS_Backend;


    // GPS_Backend overrides
    void publish(const GPS_Data *d) override;
    void update_read() override;

private:
    void send_gsof(const uint8_t *buf, const uint16_t size);

    // These packing utilities for GSOF perform a type-safe floating point byteswap.
    // They return integer types because returning floating points would involve an extra copy.
    uint64_t gsof_pack_double(const double& src) WARN_IF_UNUSED;
    uint32_t gsof_pack_float(const float& src) WARN_IF_UNUSED;
};

class GPS_NMEA : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_NMEA);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

private:

    uint8_t nmea_checksum(const char *s);
    void nmea_printf(const char *fmt, ...);
    void update_nmea(const GPS_Data *d);

};

class GPS_NOVA : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_NOVA);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

    uint32_t device_baud() const override { return 19200; }

private:

    void nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen);
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);
};

class GPS_MSP : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_MSP);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;
};

class GPS_SBP_Common : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_SBP_Common);

    using GPS_Backend::GPS_Backend;

protected:

    void sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload);

};

class GPS_SBP : public GPS_SBP_Common {
public:
    CLASS_NO_COPY(GPS_SBP);

    using GPS_SBP_Common::GPS_SBP_Common;

    void publish(const GPS_Data *d) override;

};

class GPS_SBP2 : public GPS_SBP_Common {
public:
    CLASS_NO_COPY(GPS_SBP2);

    using GPS_SBP_Common::GPS_SBP_Common;

    void publish(const GPS_Data *d) override;

};

class GPS_UBlox : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_UBlox);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

private:
    void send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
};

class GPS : public SerialDevice {
public:

    CLASS_NO_COPY(GPS);

    enum Type {
        NONE  =  0,
        UBLOX =  1,
        NMEA  =  5,
        SBP   =  6,
#if AP_SIM_GPS_FILE_ENABLED
        FILE  =  7,
#endif
        NOVA  =  8,
        SBP2  =  9,
        GSOF  = 11, // matches GPS_TYPE
        MSP   = 19,
    };

    GPS(uint8_t _instance);

    // update state
    void update();

    ssize_t write_to_autopilot(const char *p, size_t size) const override;

    uint32_t device_baud() const override;  // 0 meaning unset

private:

    uint8_t instance;

    int ext_fifo_fd;

    // The last time GPS data was written [mS]
    uint32_t last_write_update_ms;

    // last 20 samples, allowing for up to 20 samples of delay
    GPS_Data _gps_history[20];

    bool _gps_has_basestation_position;
    GPS_Data _gps_basestation_data;

    // get delayed data
    GPS_Data interpolate_data(const GPS_Data &d, uint32_t delay_ms);

    uint8_t allocated_type;
    GPS_Backend *backend;
    void check_backend_allocation();
};

}

#endif  // HAL_SIM_GPS_ENABLED
