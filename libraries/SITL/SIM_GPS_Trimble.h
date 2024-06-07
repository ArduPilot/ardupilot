#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_TRIMBLE_ENABLED

#include "SIM_GPS.h"

namespace SITL {

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

class GPS_Trimble : public GPS_Backend, public DCOL_Parser {
public:
    CLASS_NO_COPY(GPS_Trimble);

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

};

#endif  // AP_SIM_GPS_TRIMBLE_ENABLED
