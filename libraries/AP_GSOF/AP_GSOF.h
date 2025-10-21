// Trimble GSOF protocol parser library.

#pragma once

#include "AP_GSOF_config.h"

#if AP_GSOF_ENABLED

class AP_GSOF
{
public:

    static constexpr int NO_GSOF_DATA = 0;
    static constexpr int UNEXPECTED_NUM_GSOF_PACKETS = -1;

    // Parse a single byte into the buffer.
    // When enough data has arrived, it returns the number of GSOF packets parsed in the GENOUT packet.
    // If an unexpected number of GSOF packets were parsed, returns UNEXPECTED_NUM_GSOF_PACKETS.
    // This means there is no fix.
    // If it returns NO_GSOF_DATA, the parser just needs more data.
    int parse(const uint8_t temp, const uint8_t n_expected) WARN_IF_UNUSED;

    // GSOF packet numbers.
    enum msg_idx_t {
        POS_TIME = 1,
        POS = 2,
        VEL = 8,
        DOP = 9,
        POS_SIGMA = 12
    };

    // GSOF1
    struct PACKED pos_time_t {
        uint32_t time_week_ms;
        uint16_t time_week;
        uint16_t num_sats;
        uint8_t pos_flags1;
        uint8_t pos_flags2;
        uint8_t initialized_number;
    };

    pos_time_t pos_time;

    // GSOF2
    struct PACKED pos_t {
        // [rad]
        double latitude_rad;
        // [rad]
        double longitude_rad;
        // [m]
        double altitude;
    };
    pos_t position;

    // GSOF8
    struct PACKED vel_t {
        uint8_t velocity_flags;
        float horizontal_velocity;
        // True north, WGS-84 [rad]. Actually ground course, just mislabeled in the docs.
        float heading;
        // Stored in the direction from GSOF (up = positive).
        float vertical_velocity;
        // These bytes are included only if a local coordinate system is loaded.
        float local_heading;
    };
    vel_t vel;

    // GSOF9
    struct PACKED dop_t {
        float pdop;
        float hdop;
        float vdop;
        float tdop;
    };
    dop_t dop;

    // GSOF12
    struct PACKED pos_sigma_t {
        float pos_rms;
        float sigma_east;
        float sigma_north;
        float cov_east_north;
        float sigma_up;
        float semi_major_axis;
        float semi_minor_axis;
        float orientation;
        float unit_variance;
        uint16_t num_epochs;
    };
    pos_sigma_t pos_sigma;

private:

    // Parses current data and returns the number of parsed GSOF messages.
    int process_message() WARN_IF_UNUSED;

    void parse_pos_time(uint32_t a);
    void parse_pos(uint32_t a);
    void parse_vel(uint32_t a);
    void parse_dop(uint32_t a);
    void parse_pos_sigma(uint32_t a);

    struct Msg_Parser {

        enum class State {
            STARTTX = 0,
            STATUS,
            PACKETTYPE,
            LENGTH,
            DATA,
            CHECKSUM,
            ENDTX
        };

        State state {State::STARTTX};

        uint8_t status;
        uint8_t packettype;
        uint8_t length;
        uint8_t data[256];
        uint8_t checksum;
        uint8_t endtx;
        uint8_t read;

        uint8_t checksumcalc;
    } msg;

    static const uint8_t STX = 0x02;
    static const uint8_t ETX = 0x03;

    uint8_t packetcount;
    uint32_t gsofmsg_time;
};
#endif // AP_GSOF_ENABLED
