#include "SIM_config.h"

#if AP_SIM_GPS_TRIMBLE_ENABLED

#include "SIM_GPS_Trimble.h"

#include <SITL/SITL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InternalError/AP_InternalError.h>

#include <assert.h>

using namespace SITL;

void GPS_Trimble::publish(const GPS_Data *d)
{
    // This logic is to populate output buffer only with enabled channels.
    // It also only sends each channel at the configured rate.
    const uint64_t now = AP_HAL::millis();
    uint8_t buf[MAX_PAYLOAD_SIZE] = {};
    uint8_t payload_sz = 0;
    uint8_t offset = 0;
    if (channel_rates[uint8_t(Gsof_Msg_Record_Type::POSITION_TIME)] != Output_Rate::OFF){
        const auto last_time = last_publish_ms[uint8_t(Gsof_Msg_Record_Type::POSITION_TIME)];
        const auto desired_rate = channel_rates[uint8_t(Gsof_Msg_Record_Type::POSITION_TIME)];

        if (now - last_time > RateToPeriodMs(desired_rate)) {

            // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_TIME.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____25
            constexpr uint8_t GSOF_POS_TIME_LEN { 0x0A };
            // TODO magic number until SITL supports GPS bootcount based on GPSN_ENABLE
            const uint8_t bootcount = 17;

            // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Flags.html#Position%20flags%201
            enum class POS_FLAGS_1 : uint8_t {
                NEW_POSITION = 1U << 0,
                CLOCK_FIX_CALULATED = 1U << 1,
                HORIZ_FROM_THIS_POS = 1U << 2,
                HEIGHT_FROM_THIS_POS = 1U << 3,
                RESERVED_4 = 1U << 4,
                LEAST_SQ_POSITION = 1U << 5,
                RESERVED_6 = 1U << 6,
                POSITION_L1_PSEUDORANGES = 1U << 7
            };
            const uint8_t pos_flags_1 {
                uint8_t(POS_FLAGS_1::NEW_POSITION) |
                uint8_t(POS_FLAGS_1::CLOCK_FIX_CALULATED) |
                uint8_t(POS_FLAGS_1::HORIZ_FROM_THIS_POS) |
                uint8_t(POS_FLAGS_1::HEIGHT_FROM_THIS_POS) |
                uint8_t(POS_FLAGS_1::RESERVED_4) |
                uint8_t(POS_FLAGS_1::LEAST_SQ_POSITION) |
                uint8_t(POS_FLAGS_1::POSITION_L1_PSEUDORANGES)
            };

            // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Flags.html#Position%20flags%202
            enum class POS_FLAGS_2 : uint8_t {
                DIFFERENTIAL_POS = 1U << 0,
                DIFFERENTIAL_POS_PHASE_RTK = 1U << 1,
                POSITION_METHOD_FIXED_PHASE = 1U << 2,
                OMNISTAR_ACTIVE = 1U << 3,
                DETERMINED_WITH_STATIC_CONSTRAINT = 1U << 4,
                NETWORK_RTK = 1U << 5,
                DITHERED_RTK = 1U << 6,
                BEACON_DGNSS = 1U << 7,
            };

            // Simulate a GPS without RTK in SIM since there is no RTK SIM params.
            // This means these flags are unset:
            // NETWORK_RTK, DITHERED_RTK, BEACON_DGNSS
            uint8_t pos_flags_2 {0};
            if(d->have_lock) {
                pos_flags_2 |= uint8_t(POS_FLAGS_2::DIFFERENTIAL_POS);
                pos_flags_2 |= uint8_t(POS_FLAGS_2::DIFFERENTIAL_POS_PHASE_RTK);
                pos_flags_2 |= uint8_t(POS_FLAGS_2::POSITION_METHOD_FIXED_PHASE);
                pos_flags_2 |= uint8_t(POS_FLAGS_2::OMNISTAR_ACTIVE);
                pos_flags_2 |= uint8_t(POS_FLAGS_2::DETERMINED_WITH_STATIC_CONSTRAINT);
            }

            const auto gps_tow = gps_time();
            const struct PACKED gsof_pos_time {
                const uint8_t OUTPUT_RECORD_TYPE;
                const uint8_t RECORD_LEN;
                uint32_t time_week_ms;
                uint16_t time_week;
                uint8_t num_sats;
                // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Flags.html#Position%20flags%201
                uint8_t pos_flags_1;
                // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Flags.html#Position%20flags%202
                uint8_t pos_flags_2;
                uint8_t initialized_num;
            } pos_time {
                uint8_t(Gsof_Msg_Record_Type::POSITION_TIME),
                GSOF_POS_TIME_LEN,
                htobe32(gps_tow.ms),
                htobe16(gps_tow.week),
                d->have_lock ? d->num_sats : uint8_t(3),
                pos_flags_1,
                pos_flags_2,
                bootcount
            };
            static_assert(sizeof(gsof_pos_time) - (sizeof(gsof_pos_time::OUTPUT_RECORD_TYPE) + sizeof(gsof_pos_time::RECORD_LEN)) == GSOF_POS_TIME_LEN, "Trimble size check failed");

            payload_sz += sizeof(pos_time);
            memcpy(&buf[offset], &pos_time, sizeof(pos_time));
            offset += sizeof(pos_time);
        }
    }

    if (channel_rates[uint8_t(Gsof_Msg_Record_Type::LLH)] != Output_Rate::OFF){
        const auto last_time = last_publish_ms[uint8_t(Gsof_Msg_Record_Type::LLH)];
        const auto desired_rate = channel_rates[uint8_t(Gsof_Msg_Record_Type::LLH)];

        if (now - last_time > RateToPeriodMs(desired_rate)) {
            // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_LLH.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____20
            constexpr uint8_t GSOF_POS_LEN = 0x18;

            const struct PACKED gsof_pos {
                const uint8_t OUTPUT_RECORD_TYPE;
                const uint8_t RECORD_LEN;
                uint64_t lat;
                uint64_t lng;
                uint64_t alt;
            } pos {
                uint8_t(Gsof_Msg_Record_Type::LLH),
                GSOF_POS_LEN,
                gsof_pack_double(d->latitude * DEG_TO_RAD_DOUBLE),
                gsof_pack_double(d->longitude * DEG_TO_RAD_DOUBLE),
                gsof_pack_double(static_cast<double>(d->altitude))
            };
            static_assert(sizeof(gsof_pos) - (sizeof(gsof_pos::OUTPUT_RECORD_TYPE) + sizeof(gsof_pos::RECORD_LEN)) == GSOF_POS_LEN, "Trimble size check failed");

            payload_sz += sizeof(pos);
            memcpy(&buf[offset], &pos, sizeof(pos));
            offset += sizeof(pos);
        }
    }

    if (channel_rates[uint8_t(Gsof_Msg_Record_Type::VELOCITY_DATA)] != Output_Rate::OFF){
        const auto last_time = last_publish_ms[uint8_t(Gsof_Msg_Record_Type::VELOCITY_DATA)];
        const auto desired_rate = channel_rates[uint8_t(Gsof_Msg_Record_Type::VELOCITY_DATA)];

        if (now - last_time > RateToPeriodMs(desired_rate)) {
            // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Velocity.html
            // use the smaller packet by ignoring local coordinate system
            constexpr uint8_t GSOF_VEL_LEN = 0x0D;

            // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Flags.html#Velocity%20flags
            enum class VEL_FIELDS : uint8_t {
                VALID = 1U << 0,
                CONSECUTIVE_MEASUREMENTS = 1U << 1,
                HEADING_VALID = 1U << 2,
                RESERVED_3 = 1U << 3,
                RESERVED_4 = 1U << 4,
                RESERVED_5 = 1U << 5,
                RESERVED_6 = 1U << 6,
                RESERVED_7 = 1U << 7,
            };
            uint8_t vel_flags {0};
            if(d->have_lock) {
                vel_flags |= uint8_t(VEL_FIELDS::VALID);
                vel_flags |= uint8_t(VEL_FIELDS::CONSECUTIVE_MEASUREMENTS);
                vel_flags |= uint8_t(VEL_FIELDS::HEADING_VALID);
            }

            const struct PACKED gsof_vel {
                const uint8_t OUTPUT_RECORD_TYPE;
                const uint8_t RECORD_LEN;
                // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_Flags.html#Velocity%20flags
                uint8_t flags;
                uint32_t horiz_m_p_s;
                uint32_t heading_rad;
                uint32_t vertical_m_p_s;
            } vel {
                uint8_t(Gsof_Msg_Record_Type::VELOCITY_DATA),
                GSOF_VEL_LEN,
                vel_flags,
                gsof_pack_float(d->speed_2d()),
                gsof_pack_float(d->ground_track_rad()),
                // Trimble API has ambiguous direction here.
                // Intentionally narrow from double.
                gsof_pack_float(static_cast<float>(d->speedD))
            };
            static_assert(sizeof(gsof_vel) - (sizeof(gsof_vel::OUTPUT_RECORD_TYPE) + sizeof(gsof_vel::RECORD_LEN)) == GSOF_VEL_LEN, "Trimble size check failed");

            payload_sz += sizeof(vel);
            memcpy(&buf[offset], &vel, sizeof(vel));
            offset += sizeof(vel);
        }
    }
    if (channel_rates[uint8_t(Gsof_Msg_Record_Type::PDOP_INFO)] != Output_Rate::OFF){
        const auto last_time = last_publish_ms[uint8_t(Gsof_Msg_Record_Type::PDOP_INFO)];
        const auto desired_rate = channel_rates[uint8_t(Gsof_Msg_Record_Type::PDOP_INFO)];

        if (now - last_time > RateToPeriodMs(desired_rate)) {
            // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_PDOP.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____12
            constexpr uint8_t GSOF_DOP_LEN = 0x10;
            const struct PACKED gsof_dop {
                const uint8_t OUTPUT_RECORD_TYPE { uint8_t(Gsof_Msg_Record_Type::PDOP_INFO) };
                const uint8_t RECORD_LEN { GSOF_DOP_LEN };
                uint32_t pdop = htobe32(1);
                uint32_t hdop = htobe32(1);
                uint32_t vdop = htobe32(1);
                uint32_t tdop = htobe32(1);
            } dop {};
            // Check the payload size calculation in the compiler
            constexpr auto dop_size = sizeof(gsof_dop);
            static_assert(dop_size == 18, "gsof_dop must be 8 bytes");
            constexpr auto dop_record_type_size = sizeof(gsof_dop::OUTPUT_RECORD_TYPE);
            static_assert(dop_record_type_size == 1, "gsof_dop::OUTPUT_RECORD_TYPE must be 1 byte");
            constexpr auto len_size = sizeof(gsof_dop::RECORD_LEN);
            static_assert(len_size == 1, "gsof_dop::RECORD_LEN must be 1 bytes");
            constexpr auto dop_payload_size = dop_size - (dop_record_type_size + len_size);
            static_assert(dop_payload_size == GSOF_DOP_LEN, "dop_payload_size must be GSOF_DOP_LEN bytes");

            payload_sz += sizeof(dop);
            memcpy(&buf[offset], &dop, sizeof(dop));
            offset += sizeof(dop);
        }
    }
    if (channel_rates[uint8_t(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO)] != Output_Rate::OFF){
        const auto last_time = last_publish_ms[uint8_t(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO)];
        const auto desired_rate = channel_rates[uint8_t(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO)];
        if (now - last_time > RateToPeriodMs(desired_rate)) {
            // https://receiverhelp.trimble.com/oem-gnss/GSOFmessages_SIGMA.html
            constexpr uint8_t GSOF_POS_SIGMA_LEN = 0x26;
            const struct PACKED gsof_pos_sigma {
                const uint8_t OUTPUT_RECORD_TYPE { uint8_t(Gsof_Msg_Record_Type::POSITION_SIGMA_INFO) };
                const uint8_t RECORD_LEN { GSOF_POS_SIGMA_LEN };
                uint32_t pos_rms = htobe32(0);
                uint32_t sigma_e = htobe32(0);
                uint32_t sigma_n = htobe32(0);
                uint32_t cov_en = htobe32(0);
                uint32_t sigma_up = htobe32(0);
                uint32_t semi_major_axis = htobe32(0);
                uint32_t semi_minor_axis = htobe32(0);
                uint32_t orientation = htobe32(0);
                uint32_t unit_variance = htobe32(0);
                uint16_t n_epocs = htobe32(1); // Always 1 for kinematic.
            } pos_sigma {};
            static_assert(sizeof(gsof_pos_sigma) - (sizeof(gsof_pos_sigma::OUTPUT_RECORD_TYPE) + sizeof(gsof_pos_sigma::RECORD_LEN)) == GSOF_POS_SIGMA_LEN, "Trimble size check failed");
            payload_sz += sizeof(pos_sigma);
            memcpy(&buf[offset], &pos_sigma, sizeof(pos_sigma));
            offset += sizeof(pos_sigma);
        }
    }

    assert(offset == payload_sz);

    // Don't send empy frames when all channels are disabled;
    if (payload_sz > 0) {
        send_gsof(buf, payload_sz);
    }

}

bool DCOL_Parser::dcol_parse(const char data_in) {
    bool ret = false;
    switch (parse_state) {
    case Parse_State::WAITING_ON_STX:
        if (data_in == STX) {
            reset();
            parse_state = Parse_State::WAITING_ON_STATUS;
        }
        break;
    case Parse_State::WAITING_ON_STATUS:
        if (data_in != (uint8_t)Status::OK) {
            // Invalid, status should always be OK.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        } else {
            status = static_cast<Status>(data_in);
            parse_state = Parse_State::WAITING_ON_PACKET_TYPE;
        }
        break;
    case Parse_State::WAITING_ON_PACKET_TYPE:
        if (data_in == (uint8_t)Packet_Type::COMMAND_APPFILE) {
            packet_type = Packet_Type::COMMAND_APPFILE;
        } else {
            // Unsupported command in this simulator.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
        parse_state = Parse_State::WAITING_ON_LENGTH;
        break;
    case Parse_State::WAITING_ON_LENGTH:
        expected_payload_length = data_in;
        parse_state = Parse_State::WAITING_ON_PACKET_DATA;
        break;
    case Parse_State::WAITING_ON_PACKET_DATA:
        payload[cur_payload_idx] = data_in;
        if (++cur_payload_idx == expected_payload_length) {
            parse_state = Parse_State::WAITING_ON_CSUM;
        }
        break;
    case Parse_State::WAITING_ON_CSUM:
        expected_csum = data_in;
        parse_state = Parse_State::WAITING_ON_ETX;
        break;
    case Parse_State::WAITING_ON_ETX:
        if (data_in != ETX) {
            reset();
        }
        if (!valid_csum()) {
            // GSOF driver sent a packet with invalid CSUM.
            // In real GSOF driver, the packet is simply ignored with no reply.
            // In the SIM, treat this as a coding error.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        } else {
            ret = parse_payload();
        }
        reset();
        break;
    }

    return ret;
}

uint32_t DCOL_Parser::RateToPeriodMs(const Output_Rate rate) {
    uint32_t min_period_ms = 0;
    switch (rate) {
        case Output_Rate::OFF:
            min_period_ms = 0;
            break;
        case Output_Rate::FREQ_10_HZ:
            min_period_ms = 100;
            break;
        case Output_Rate::FREQ_50_HZ:
            min_period_ms = 20;
            break;
        case Output_Rate::FREQ_100_HZ:
            min_period_ms = 10;
            break;
    }
    return min_period_ms;
}


bool DCOL_Parser::valid_csum() {
    uint8_t sum = (uint8_t)status;
    sum += (uint8_t)packet_type;
    sum += expected_payload_length;
    sum += crc_sum_of_bytes(payload, expected_payload_length);
    return sum == expected_csum;
}

bool DCOL_Parser::parse_payload() {
    bool success = false;
    if (packet_type == Packet_Type::COMMAND_APPFILE) {
        success = parse_cmd_appfile();
    }

    return success;
}

bool DCOL_Parser::parse_cmd_appfile() {
    // A file control info block contains:
    // * version
    // * device type
    // * start application file flag
    // * factory settings flag
    constexpr uint8_t file_control_info_block_sz = 4;
    // An appfile header contains:
    // * transmisison number
    // * page index
    // * max page index
    constexpr uint8_t appfile_header_sz = 3;
    constexpr uint8_t min_cmd_appfile_sz = file_control_info_block_sz + appfile_header_sz;
    if (expected_payload_length < min_cmd_appfile_sz) {
        return false;
    }

    // For now, ignore appfile_trans_num, return success regardless.
    // If the driver doesn't send the right value, it's not clear what the behavior is supposed to be.
    // Also would need to handle re-sync.
    // For now, just store it for debugging.
    appfile_trans_num = payload[0];

    // File control information block parsing:
    // https://receiverhelp.trimble.com/oem-gnss/ICD_Subtype_Command64h_FileControlInfo.html
    constexpr uint8_t expected_app_file_spec_version = 0x03;
    constexpr uint8_t file_ctrl_idx = appfile_header_sz;
    if (payload[file_ctrl_idx] != expected_app_file_spec_version) {
        // Only version 3 is supported at this time.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    constexpr uint8_t expected_dev_type = 0x00;
    if (payload[file_ctrl_idx+1] != expected_dev_type) {
        // Only "all" device type is supported.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    constexpr uint8_t expected_start_flag = 0x01;
    if (payload[file_ctrl_idx+2] != expected_start_flag) {
        // Only "immediate" start type is supported.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }


    constexpr uint8_t expected_factory_settings_flag = 0x00;
    if (payload[file_ctrl_idx+3] != expected_factory_settings_flag) {
        // Factory settings restore before appfile is not supported.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    constexpr uint8_t app_file_records_idx = appfile_header_sz + file_control_info_block_sz;
    const uint8_t record_type = payload[app_file_records_idx];
    if (record_type == (uint8_t)Appfile_Record_Type::SERIAL_PORT_BAUD_RATE_FORMAT) {
        // Serial port baud/format
        // https://receiverhelp.trimble.com/oem-gnss/ICD_Command64h_AppFile_SerialPort.html

        // Ignore serial port index (COM Port) since there's only one in SITL.
        // Ignore baud rate because you can't change baud yet in SITL.
        // Ignore parity because it can't be changed in SITL.
        // Ignore flow control because it's not yet in SITL.
    } else if (record_type == (uint8_t)Appfile_Record_Type::OUTPUT_MESSAGE){
        // Output Message
        // https://receiverhelp.trimble.com/oem-gnss/ICD_Command64h_AppFile_Output.html


        // Ignore record length to save flash.
        // Ignore port index since SITL is only one port.
        if (payload[app_file_records_idx + 2] == (uint8_t)(Output_Msg_Msg_Type::GSOF)) {
            const auto gsof_submessage_type = payload[app_file_records_idx + 6];
            const auto rate = payload[app_file_records_idx + 4];
            if (rate == (uint8_t)Output_Rate::OFF) {
                channel_rates[gsof_submessage_type] = static_cast<Output_Rate>(rate);
            } else if (rate == (uint8_t)Output_Rate::FREQ_10_HZ) {
                channel_rates[gsof_submessage_type] = static_cast<Output_Rate>(rate);
            } else if (rate == (uint8_t)Output_Rate::FREQ_50_HZ) {
                channel_rates[gsof_submessage_type] = static_cast<Output_Rate>(rate);
            } else if (rate == (uint8_t)Output_Rate::FREQ_100_HZ) {
                channel_rates[gsof_submessage_type] = static_cast<Output_Rate>(rate);
            } else {
                // Unsupported GSOF rate in SITL.
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
        } else {
            // Only some data output protocols are supported in SITL.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }

    } else {
        // Other application file packets are not yet supported.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    return true;
}

void DCOL_Parser::reset() {
    cur_payload_idx = 0;
    expected_payload_length = 0;
    parse_state = Parse_State::WAITING_ON_STX;
    // To be pedantic, one could zero the bytes in the payload,
    // but this is skipped because it's extra CPU.

    // Note - appfile_trans_number is intended to persist over parser resets.
}


void GPS_Trimble::send_gsof(const uint8_t *buf, const uint16_t size)
{
    // All Trimble "Data Collector" packets, including GSOF, are comprised of three fields:
    // * A fixed-length packet header (dcol_header)
    // * A variable-length data frame (buf)
    // * A fixed-length packet trailer (dcol_trailer)
    // Reference: // https://receiverhelp.trimble.com/oem-gnss/index.html#API_DataCollectorFormatPacketStructure.html?TocPath=API%2520Documentation%257CData%2520collector%2520format%2520packets%257CData%2520collector%2520format%253A%2520packet%2520structure%257C_____0

    // status bitfield
    // https://receiverhelp.trimble.com/oem-gnss/index.html#API_ReceiverStatusByte.html?TocPath=API%2520Documentation%257CData%2520collector%2520format%2520packets%257CData%2520collector%2520format%253A%2520packet%2520structure%257C_____1
    const uint8_t STATUS = 0xa8;
    const uint8_t PACKET_TYPE = 0x40; // Report Packet 40h (GENOUT)

    // Before writing the GSOF data buffer, the GSOF header needs added between the DCOL header and the payload data frame.
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_GSOF.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____2

    static uint8_t TRANSMISSION_NUMBER = 0; // Functionally, this is a sequence number
    // Most messages, even GSOF49, only take one page. For SIM, assume it.
    assert(size < 0xFA); // GPS SIM doesn't yet support paging
    constexpr uint8_t PAGE_INDEX = 0;
    constexpr uint8_t MAX_PAGE_INDEX = 0;
    const uint8_t gsof_header[3] = {
        TRANSMISSION_NUMBER,
        PAGE_INDEX,
        MAX_PAGE_INDEX,
    };
    ++TRANSMISSION_NUMBER;

    // A captured GSOF49 packet from BD940 has LENGTH field set to 0x6d = 109 bytes.
    // A captured GSOF49 packet from BD940 has total bytes of 115 bytes.
    // Thus, the following 5 bytes are not counted.
    // 1) STX
    // 2) STATUS
    // 3) PACKET TYPE
    // 4) LENGTH
    // 5) CHECKSUM
    // 6) ETX
    // This aligns with manual's idea of data bytes:
    // "Each message begins with a 4-byte header, followed by the bytes of data in each packet. The packet ends with a 2-byte trailer."
    // Thus, for this implementation with single-page single-record per DCOL packet,
    // the length is simply the sum of data packet size, the gsof_header size.
    const uint8_t length = size + sizeof(gsof_header);
    const uint8_t dcol_header[4] {
        STX,
        STATUS,
        PACKET_TYPE,
        length
    };


    // Sum bytes (status + type + length + data bytes) and modulo 256 the summation
    // Because it's a uint8, use natural overflow
    uint8_t csum = STATUS + PACKET_TYPE + length;
    for (size_t i = 0; i < ARRAY_SIZE(gsof_header); i++) {
        csum += gsof_header[i];
    }
    for (size_t i = 0; i < size; i++) {
        csum += buf[i];
    }

    const uint8_t dcol_trailer[2] = {
        csum,
        ETX
    };

    write_to_autopilot((char*)dcol_header, sizeof(dcol_header));
    write_to_autopilot((char*)gsof_header, sizeof(gsof_header));
    write_to_autopilot((char*)buf, size);
    write_to_autopilot((char*)dcol_trailer, sizeof(dcol_trailer));
    const uint8_t total_size = sizeof(dcol_header) + sizeof(gsof_header) + size + sizeof(dcol_trailer);
     // Validate length based on everything but DCOL h
    if(dcol_header[3] != total_size - (sizeof(dcol_header) +  sizeof(dcol_trailer))) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

uint64_t GPS_Trimble::gsof_pack_double(const double& src)
{
    uint64_t dst;
    static_assert(sizeof(src) == sizeof(dst), "src and dst must have equal size");
    memcpy(&dst, &src, sizeof(dst));
    dst = htobe64(dst);
    return dst;
}

uint32_t GPS_Trimble::gsof_pack_float(const float& src)
{
    uint32_t dst;
    static_assert(sizeof(src) == sizeof(dst), "src and dst must have equal size");
    memcpy(&dst, &src, sizeof(dst));
    dst = htobe32(dst);
    return dst;
}

void GPS_Trimble::update_read() {
    // Technically, the max command is slightly larger.
    // This will only slightly slow the response for packets that big.
    char c[MAX_PAYLOAD_SIZE];
    const auto n_read = read_from_autopilot(c, MAX_PAYLOAD_SIZE);
    if (n_read > 0) {
        for (uint8_t i = 0; i < n_read; i++) {
            if (dcol_parse(c[i])) {
                constexpr uint8_t response[1] = {(uint8_t)Command_Response::ACK};
                write_to_autopilot((char*)response, sizeof(response));
            }
            // TODO handle NACK for failure
        }
    }
}

#endif  // AP_SIM_GPS_TRIMBLE_ENABLED
