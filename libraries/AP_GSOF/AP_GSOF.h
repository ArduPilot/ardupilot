// Trimble GSOF protocol parser library.
// https://receiverhelp.trimble.com/oem-gnss/gsof-messages-overview.html
// GSOF packets are assumed to be inside DCOL packets.

#pragma once

#include "AP_GSOF_config.h"
#include <AP_Common/Bitmask.h>
#include <AP_GPS/AP_GPS_FixType.h>

#if AP_GSOF_ENABLED

class AP_GSOF
{
public:

    static constexpr int NO_GSOF_DATA = 0;
    static constexpr int PARSED_GSOF_DATA = 1;
    static constexpr uint8_t MAX_PACKET_SIZE {255};

    // A type alias for which packets have been parsed.
    using MsgTypes = Bitmask<71>; // one more than GSOF70.

    // Parse a single byte into the buffer.
    // When enough data has arrived, it populates a bitmask of which GSOF packets were parsed in the GENOUT packet and returns PARSED_GSOF_DATA.
    // If it returns NO_GSOF_DATA, the parser just needs more data.
    int parse(const uint8_t temp, MsgTypes& parsed_msgs) WARN_IF_UNUSED;

    // GSOF packet numbers.
    enum msg_idx_t {
        POS_TIME = 1,
        POS = 2,
        VEL = 8,
        DOP = 9,
        POS_SIGMA = 12,
        INS_FULL_NAV = 49,
        INS_RMS = 50,
        LLH_MSL = 70,
    };

    enum class ImuAlignmentStatus {
        GPS_ONLY = 0,
        COARSE_LEVELING = 1,
        DEGRADED = 2,
        ALIGNED = 3,
        FULL_NAV = 4
    };

    enum class GnssStatus {
        FIX_NOT_AVAILABLE = 0,
        GNSS_SPS_MODE = 1,
        DGPS_SPS_MODE = 2,
        GNSS_PPS_MODE = 3,
        FIXED_RTK_MODE = 4,
        FLOAT_RTK_MODE = 5,
        DR_MODE = 6
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

    // GSOF49
    struct PACKED ins_full_nav_t {
        // GPS week number since Jan 1980
        uint16_t gps_week;
        // GPS Time in msec of current week
        uint32_t gps_time_ms;
        ImuAlignmentStatus imu_alignment_status;
        GnssStatus gnss_status;
        double latitude;
        double longitude;
        // Altitude is in ITRF 2020.
        double altitude;
        float vel_n;
        float vel_e;
        float vel_d;
        float speed;
        double roll_deg;
        double pitch_deg;
        double heading_deg;
        double track_angle_deg;
        // [deg/s, longitudinal axis]
        float ang_rate_x;
        // [deg/s, transverse axis]
        float ang_rate_y;
        // [m/s, down axis]
        float ang_rate_z;
        // [m/s^2, longitudinal axis]
        float acc_x;
        // [m/s^2, transverse axis]
        float acc_y;
        // [m/s^2, down axis]
        float acc_z;
    };
    ins_full_nav_t ins_full_nav;

    // GSOF50
    struct PACKED ins_rms_t {
        // GPS week number since Jan 1980
        uint16_t gps_week;
        // GPS Time in msec of current week
        uint32_t gps_time_ms;
        ImuAlignmentStatus imu_alignment_status;
        GnssStatus gnss_status;
        // North Position RMS [meters]
        float pos_rms_n;
        // East Position RMS [meters]
        float pos_rms_e;
        // Down Position RMS [meters]
        float pos_rms_d;
        // North Velocity RMS [meters/sec]
        float vel_rms_n;
        // East Velocity RMS [meters/sec]
        float vel_rms_e;
        // Down Velocity RMS [meters/sec]
        float vel_rms_d;
        // Roll RMS [deg]
        float roll_rms_deg;
        // Pitch RMS [deg]
        float pitch_rms_deg;
        // Yaw RMS [deg]
        float yaw_rms_deg;
    };
    ins_rms_t ins_rms;

    // GSOF70
    struct PACKED llh_msl_t {
        double latitude;
        double longitude;
        double altitude_msl;
        char model[12];
    };
    llh_msl_t llh_msl;

private:

    // Parses current data.
    // Returns true if, and only if, the expected packets were parsed.
    // Unexpected/unparsed data will make this return false.
    bool process_message(MsgTypes& parsed_msgs) WARN_IF_UNUSED;

    void parse_pos_time(uint32_t a);
    void parse_pos(uint32_t a);
    void parse_vel(uint32_t a);
    void parse_dop(uint32_t a);
    void parse_pos_sigma(uint32_t a);
    void parse_ins_full_nav(uint32_t a);
    void parse_ins_rms(uint32_t a);
    void parse_llh_msl(uint32_t a);


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
        uint8_t data[MAX_PACKET_SIZE];
        uint8_t checksum;
        uint8_t endtx;
        uint8_t read;

        uint8_t checksumcalc;
    } msg;

    static const uint8_t STX = 0x02;
    static const uint8_t ETX = 0x03;
};
#endif // AP_GSOF_ENABLED
