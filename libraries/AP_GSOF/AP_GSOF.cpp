#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_GSOF_config.h"
#include "AP_GSOF.h"

#if AP_GSOF_ENABLED



#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define gsof_DEBUGGING 0

#if gsof_DEBUGGING
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
AP_GSOF::parse(const uint8_t temp, const uint8_t n_expected)
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
            const auto n_processed = process_message();
            if (n_processed != n_expected ) {
                return UNEXPECTED_NUM_GSOF_PACKETS;
            }
            return n_processed;
        }
        break;
    case Msg_Parser::State::ENDTX:
        msg.endtx = temp;
        msg.state = Msg_Parser::State::STARTTX;
        break;
    }

    return NO_GSOF_DATA;
}

double
AP_GSOF::SwapDouble(const uint8_t* src, const uint32_t pos) const
{
    union {
        double d;
        char bytes[sizeof(double)];
    } doubleu;
    doubleu.bytes[0] = src[pos + 7];
    doubleu.bytes[1] = src[pos + 6];
    doubleu.bytes[2] = src[pos + 5];
    doubleu.bytes[3] = src[pos + 4];
    doubleu.bytes[4] = src[pos + 3];
    doubleu.bytes[5] = src[pos + 2];
    doubleu.bytes[6] = src[pos + 1];
    doubleu.bytes[7] = src[pos + 0];

    return doubleu.d;
}

float
AP_GSOF::SwapFloat(const uint8_t* src, const uint32_t pos) const
{
    union {
        float f;
        char bytes[sizeof(float)];
    } floatu;
    floatu.bytes[0] = src[pos + 3];
    floatu.bytes[1] = src[pos + 2];
    floatu.bytes[2] = src[pos + 1];
    floatu.bytes[3] = src[pos + 0];

    return floatu.f;
}

uint32_t
AP_GSOF::SwapUint32(const uint8_t* src, const uint32_t pos) const
{
    uint32_t u;
    memcpy(&u, &src[pos], sizeof(u));
    return be32toh(u);
}

uint16_t
AP_GSOF::SwapUint16(const uint8_t* src, const uint32_t pos) const
{
    uint16_t u;
    memcpy(&u, &src[pos], sizeof(u));
    return be16toh(u);
}

int
AP_GSOF::process_message(void)
{
    if (msg.packettype == 0x40) { // GSOF
        // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_TIME.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____25
#if gsof_DEBUGGING
        const uint8_t trans_number = msg.data[0];
        const uint8_t pageidx = msg.data[1];
        const uint8_t maxpageidx = msg.data[2];

        Debug("GSOF page: %u of %u (trans_number=%u)",
              pageidx, maxpageidx, trans_number);
#endif

        // This counts up the number of received packets.
        int valid = 0;

        // want 1 2 8 9 12
        for (uint32_t a = 3; a < msg.length; a++) {
            const uint8_t output_type = msg.data[a];
            a++;
            const uint8_t output_length = msg.data[a];
            a++;
            // TODO handle corruption on output_length causing buffer overrun?

            switch (output_type) {
            case POS_TIME:
                parse_pos_time(a);
                valid++;
                break;
            case POS:
                parse_pos(a);
                valid++;
                break;
            case VEL:
                parse_vel(a);
                valid++;
                break;
            case DOP:
                parse_dop(a);
                valid++;
                break;
            case POS_SIGMA:
                parse_pos_sigma(a);
                valid++;
                break;
            default:
                break;
            }

            a += output_length - 1u;
        }

        return valid;
    }

    // No GSOF packets.
    return NO_GSOF_DATA;
}

void AP_GSOF::parse_pos_time(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_TIME.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____25
    pos_time.time_week_ms = SwapUint32(msg.data, a);
    pos_time.time_week = SwapUint16(msg.data, a + 4);
    pos_time.num_sats = msg.data[a + 6];
    pos_time.pos_flags1 = msg.data[a + 7];
    pos_time.pos_flags2 = msg.data[a + 8];
}

void AP_GSOF::parse_pos(uint32_t a)
{
    // This packet is not documented in Trimble's receiver help as of May 18, 2023
    position.latitude_rad = SwapDouble(msg.data, a);
    position.longitude_rad = SwapDouble(msg.data, a + 8);
    position.altitude = SwapDouble(msg.data, a + 16);
}

void AP_GSOF::parse_vel(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_Velocity.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____32
    vel.velocity_flags = msg.data[a];

    constexpr uint8_t BIT_VELOCITY_VALID = 0;
    if (BIT_IS_SET(vel.velocity_flags, BIT_VELOCITY_VALID)) {
        vel.horizontal_velocity = SwapFloat(msg.data, a + 1);
        vel.vertical_velocity = SwapFloat(msg.data, a + 9);
    }

    constexpr uint8_t BIT_HEADING_VALID = 2;
    if (BIT_IS_SET(vel.velocity_flags, BIT_HEADING_VALID)) {
        vel.heading = SwapFloat(msg.data, a + 5);
    }
}

void AP_GSOF::parse_dop(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_PDOP.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____12
    // Skip pdop.
    dop.hdop = SwapFloat(msg.data, a + 4);
}

void AP_GSOF::parse_pos_sigma(uint32_t a)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_SIGMA.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____24
    // Skip pos_rms
    pos_sigma.sigma_east = SwapFloat(msg.data, a + 4);
    pos_sigma.sigma_north = SwapFloat(msg.data, a + 8);
    pos_sigma.sigma_up = SwapFloat(msg.data, a + 16);
}
#endif // AP_GSOF_ENABLED

