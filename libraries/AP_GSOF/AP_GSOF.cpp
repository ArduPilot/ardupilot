#include "AP_GSOF_config.h"

#if AP_GSOF_ENABLED

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GSOF/AP_GSOF.h>

#define gsof_DEBUGGING 0
#if gsof_DEBUGGING
extern const AP_HAL::HAL& hal;
# define Debug(fmt, args ...)                  \
do {                                            \
    hal.console->printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
# define Debug(fmt, args ...)
#endif

int
AP_GSOF::parse(const uint8_t temp, MsgTypes& parsed_msgs)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#API_DataCollectorFormatPacketStructure.html
    switch (msg.state) {
    default:
    case Msg_Parser::State::STARTTX:
        if (temp == STX) {
            msg.state = Msg_Parser::State::STATUS;
            msg.read = 0;
            msg.checksumcalc = 0;
        }
        break;
    case Msg_Parser::State::STATUS:
        msg.status = temp;
        msg.state = Msg_Parser::State::PACKETTYPE;
        msg.checksumcalc += temp;
        break;
    case Msg_Parser::State::PACKETTYPE:
        msg.packettype = temp;
        msg.state = Msg_Parser::State::LENGTH;
        msg.checksumcalc += temp;
        break;
    case Msg_Parser::State::LENGTH:
        msg.length = temp;
        msg.state = Msg_Parser::State::DATA;
        msg.checksumcalc += temp;
        break;
    case Msg_Parser::State::DATA:
        msg.data[msg.read] = temp;
        msg.read++;
        msg.checksumcalc += temp;
        if (msg.read >= msg.length) {
            msg.state = Msg_Parser::State::CHECKSUM;
        }
        break;
    case Msg_Parser::State::CHECKSUM:
        msg.checksum = temp;
        msg.state = Msg_Parser::State::ENDTX;
        if (msg.checksum == msg.checksumcalc) {
            if (!process_message(parsed_msgs)) {
                return NO_GSOF_DATA;
            }
            return PARSED_GSOF_DATA;
        }
        break;
    case Msg_Parser::State::ENDTX:
        msg.endtx = temp;
        msg.state = Msg_Parser::State::STARTTX;
        break;
    }

    return NO_GSOF_DATA;
}

bool
AP_GSOF::process_message(MsgTypes& parsed_msgs)
{
    if (msg.packettype == 0x40) { // GSOF
        // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_TIME.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____25

        for (uint32_t a = 3; a < msg.length; a++) {
            const uint8_t output_type = msg.data[a];

            if (output_type >= parsed_msgs.size()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                AP_HAL::panic("Invalid output type.");
#else
                return false;
#endif
            }

            parsed_msgs.set(output_type);
            a++;
            const uint8_t output_length = msg.data[a];
            a++;
            // TODO handle corruption on output_length causing buffer overrun?

            switch (output_type) {
            case POS_TIME:
                parse_pos_time(a);
                break;
            case POS:
                parse_pos(a);
                break;
            case VEL:
                parse_vel(a);
                break;
            case DOP:
                parse_dop(a);
                break;
            case POS_SIGMA:
                parse_pos_sigma(a);
                break;
            case INS_FULL_NAV:
                parse_ins_full_nav(a);
                break;
            case INS_RMS:
                parse_ins_rms(a);
                break;
            case LLH_MSL:
                parse_llh_msl(a);
                break;
            default:
                // TODO log warning unparsed packet.
                break;
            }

            a += output_length - 1u;
        }

        return true;
    }

    // No GSOF packets.
    return false;
}

void AP_GSOF::parse_pos_time(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-time.html
    pos_time.time_week_ms = be32toh_ptr(msg.data + a);
    pos_time.time_week = be32toh_ptr(msg.data + a + 4);
    pos_time.num_sats = msg.data[a + 6];
    pos_time.pos_flags1 = msg.data[a + 7];
    pos_time.pos_flags2 = msg.data[a + 8];
}

void AP_GSOF::parse_pos(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-llh.html
    position.latitude_rad = be64todouble_ptr(msg.data, a);
    position.longitude_rad = be64todouble_ptr(msg.data, a + 8);
    // Altitude is "Height from WGS-84 datum" -> Likely ellipsoid
    position.altitude = be64todouble_ptr(msg.data, a + 16);
}

void AP_GSOF::parse_vel(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-velocity.html

    constexpr uint8_t BIT_VELOCITY_VALID = 0;
    if (BIT_IS_SET(vel.velocity_flags, BIT_VELOCITY_VALID)) {
        vel.horizontal_velocity = be32tofloat_ptr(msg.data, a + 1);
        vel.vertical_velocity = be32tofloat_ptr(msg.data, a + 9);
    }

    constexpr uint8_t BIT_HEADING_VALID = 2;
    if (BIT_IS_SET(vel.velocity_flags, BIT_HEADING_VALID)) {
        vel.heading = be32tofloat_ptr(msg.data, a + 5);
    }
}

void AP_GSOF::parse_dop(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-pdop.html
    // Skip pdop.
    dop.hdop = be32tofloat_ptr(msg.data, a + 4);
}

void AP_GSOF::parse_pos_sigma(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-sigma.html
    // Skip pos_rms
    pos_sigma.sigma_east = be32tofloat_ptr(msg.data, a + 4);
    pos_sigma.sigma_north = be32tofloat_ptr(msg.data, a + 8);
    pos_sigma.sigma_up = be32tofloat_ptr(msg.data, a + 16);
}

void AP_GSOF::parse_ins_full_nav(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-ins-full-nav.html
    ins_full_nav.gps_week = be16toh_ptr(msg.data + a);
    ins_full_nav.gps_time_ms = be32toh_ptr(msg.data + a + 2);
    ins_full_nav.imu_alignment_status = ImuAlignmentStatus(msg.data[a + 6]);
    ins_full_nav.gnss_status = GnssStatus(msg.data[a + 7]);
    ins_full_nav.latitude = be64todouble_ptr(msg.data, a + 8);
    ins_full_nav.longitude = be64todouble_ptr(msg.data, a + 16);
    ins_full_nav.altitude = be64todouble_ptr(msg.data, a + 24);
    ins_full_nav.vel_n = be32tofloat_ptr(msg.data, a + 32);
    ins_full_nav.vel_e = be32tofloat_ptr(msg.data, a + 36);
    ins_full_nav.vel_d = be32tofloat_ptr(msg.data, a + 40);
    ins_full_nav.speed = be32tofloat_ptr(msg.data, a + 44);
    ins_full_nav.roll_deg = be64todouble_ptr(msg.data, a + 48);
    ins_full_nav.pitch_deg = be64todouble_ptr(msg.data, a + 56);
    ins_full_nav.heading_deg = be64todouble_ptr(msg.data, a + 64);
    ins_full_nav.track_angle_deg = be64todouble_ptr(msg.data, a + 72);
    // Remaining data is currently unused.
}

void AP_GSOF::parse_ins_rms(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-ins-rms.html
    ins_rms.gps_week = be16toh_ptr(msg.data + a);
    ins_rms.gps_time_ms = be32toh_ptr(msg.data + a + 2);
    ins_rms.imu_alignment_status = ImuAlignmentStatus(msg.data[a + 6]);
    ins_rms.gnss_status = GnssStatus(msg.data[a + 7]);
}

void AP_GSOF::parse_llh_msl(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/gsof-messages-llmsl.html
    llh_msl.latitude = RAD_TO_DEG_DOUBLE * be64todouble_ptr(msg.data, a);
    llh_msl.longitude = RAD_TO_DEG_DOUBLE * be64todouble_ptr(msg.data, a + 8);
    llh_msl.altitude_msl = be64todouble_ptr(msg.data, a + 16);
    // Assume the model is EGM96.
}

#endif // AP_GSOF_ENABLED

