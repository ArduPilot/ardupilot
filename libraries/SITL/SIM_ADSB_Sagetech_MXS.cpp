#include "SIM_config.h"

#if AP_SIM_ADSB_SAGETECH_MXS_ENABLED

#include "SIM_ADSB_Sagetech_MXS.h"

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>
#include "SITL.h"
#include <ctype.h>
#include "SIM_Aircraft.h"
#include "SIM_ADSB.h"

#include <errno.h>

static const uint8_t FIRST_BYTE { 0xAA };

using namespace SITL;

void ADSB_Sagetech_MXS::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<buflen; i++) {
        if ((uint8_t)msg.buffer[i] == FIRST_BYTE) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(msg.buffer, &msg.buffer[i], buflen-i);
    buflen = buflen - i;
}

void ADSB_Sagetech_MXS::update_serial_input()
{
    const ssize_t n = read_from_autopilot(&msg.buffer[buflen], ARRAY_SIZE(msg.buffer) - buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }


    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "read (%u) bytes", (unsigned)n);

    switch (input_state) {
    case InputState::WANT_START_BYTE:
        move_preamble_in_buffer();
        if (buflen == 0) {
            return;
        }
        set_input_state(InputState::WANT_PREAMBLE);
        FALLTHROUGH;
    case InputState::WANT_PREAMBLE:
        if (buflen < sizeof(msg.preamble)) {
            return;
        }
        set_input_state(InputState::WANT_PAYLOAD);
        FALLTHROUGH;
    case InputState::WANT_PAYLOAD: {
        // 1 preamble, 2 flags, 1 msg + payload
        const uint8_t want_len = 4 + msg.preamble.payload_length;
        if (buflen < want_len) {
            return;
        }
        set_input_state(InputState::WANT_CHECKSUM);
        FALLTHROUGH;
    }
    case InputState::WANT_CHECKSUM: {
        // 1 start byte, 1 msg type, 1 msgid, 1 payload len
        const uint8_t want_len = sizeof(msg.preamble) + msg.preamble.payload_length + 1;
        if (buflen < want_len) {
            return;
        }
        const uint8_t received_checksum = msg.buffer[want_len-1];

        uint8_t calculated_checksum = 0;
        for (uint8_t i=0; i<sizeof(msg.preamble)+msg.preamble.payload_length; i++) {
            calculated_checksum += msg.buffer[i];
        }

        if (calculated_checksum == received_checksum) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sim-MXS: Got one (%u)!", (unsigned)msg.preamble.msgtype);
            handle_message();
            move_preamble_in_buffer(want_len);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sim-MXS: Bad checksum");
            move_preamble_in_buffer(1);
        }
        // consume these bytes
        set_input_state(InputState::WANT_PREAMBLE);
        return;
    }
    }
}

bool ADSB_Sagetech_MXS::_handle_message()
{
    switch (msg.preamble.msgtype) {
    case MsgType::INSTALLATION:
        return handle_message(msg.packed_installation.msg);
    case MsgType::FLIGHTID:
        return handle_message(msg.packed_flightidmessage.msg);
    case MsgType::OPMSG:
        return handle_message(msg.packed_operating.msg);
    case MsgType::GPS:
        return handle_message(msg.packed_gps.msg);
    case MsgType::DATAREQ:
        return handle_message(msg.packed_data_req.msg);
    case MsgType::TARGETREQUEST:
        return handle_message(msg.packed_target_req.msg);

    case MsgType::ACK:
    case MsgType::STATEVECTORREPORT:
    case MsgType::MODESTATUSREPORT:
        // we should never receive one of these?
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    }

    // we should handle all messages:
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);

    return false;
}

void ADSB_Sagetech_MXS::handle_message()
{
    if (!_handle_message()) {
        return;
    }

    // store ack info for sending an ack later:
    ack_info.ackType = msg.preamble.msgtype;
    ack_info.ackId = msg.preamble.msgid;
    ack_info.send = true;
}

bool ADSB_Sagetech_MXS::handle_message(const SITL::ADSB_Sagetech_MXS::Installation& opmsg)
{
    ASSERT_STORAGE_SIZE(Installation, 36);

    if (operating_mode != OperatingMode::OFF &&
        operating_mode != OperatingMode::MAINTENANCE) {
        // see page 10 - ignored if not in one of those.  We could
        // return silently here if there are race conditions
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // do something!

    return true;
}

void ADSB_Sagetech_MXS::assert_good_flight_id()
{
    static_assert(sizeof(flight_id) == 8, "correct storage size");

    bool space_seen = false;
    for (uint8_t i=0; i<sizeof(flight_id); i++) {
        // this doesn't seem right; they really allow any ASCII
        // character?  vertical tabs, for example?!  The standard is
        // ~US$750.  Null (0x0) is a valid ascii character!
        if (!isascii(flight_id[i])) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
        if (flight_id[i] == ' ') {
            space_seen = true;
        }
        if (space_seen && flight_id[i] != ' ') {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }
}

bool ADSB_Sagetech_MXS::handle_message(const SITL::ADSB_Sagetech_MXS::FlightIDMessage& _msg)
{
    ASSERT_STORAGE_SIZE(FlightIDMessage, 12);

    // do something!
    flight_id_set_time_ms = AP_HAL::millis();
    memcpy(flight_id, _msg.flight_id, sizeof(flight_id));
    assert_good_flight_id();

    return true;
}

bool ADSB_Sagetech_MXS::handle_message(const SITL::ADSB_Sagetech_MXS::Operating& _msg)
{
    ASSERT_STORAGE_SIZE(Operating, 12);

    // do something!

    return true;
}

#define ORD(x) (x - '0')

// buffer contains DDDMM.MMMMM (degrees, minutes including fraction)
double ADSB_Sagetech_MXS::lon_string_to_double(const uint8_t *str)
{
    return ORD(str[0]) * 100 + lat_string_to_double(&str[1]);
}

// buffer contains DDMM.MMMMM (degrees, minutes including fraction)
double ADSB_Sagetech_MXS::lat_string_to_double(const uint8_t *str)
{
    double degrees = ORD(str[0]) * 10 + ORD(str[1]);
    double minutes = ORD(str[2]) * 10.0 + ORD(str[3]) + ORD(str[5])*0.1 + ORD(str[6])*0.01 + ORD(str[7]) * 0.001 + ORD(str[8]) * 0.0001 + ORD(str[9]) * 0.00001;
    return degrees + minutes/60.0;
}

#undef ORD

bool ADSB_Sagetech_MXS::handle_message(const SITL::ADSB_Sagetech_MXS::GPS& _msg)
{
    ASSERT_STORAGE_SIZE(GPS, 63);

    // store data to transmit via ADSB
    info_from_vehicle.gps.lat = lat_string_to_double(_msg.latitude);
    info_from_vehicle.gps.lng = lon_string_to_double(_msg.longitude);

    return true;
}

bool ADSB_Sagetech_MXS::handle_message(const SITL::ADSB_Sagetech_MXS::DataRequest& _msg)
{
    ASSERT_STORAGE_SIZE(DataRequest, 4);

    // handle request to send data to vehicle.  Note that the
    // specification says (on page 32) that the ack is sent before the
    // data.

    return true;
}

bool ADSB_Sagetech_MXS::handle_message(const SITL::ADSB_Sagetech_MXS::TargetRequest& _msg)
{
    ASSERT_STORAGE_SIZE(TargetRequest, 7);

    // handle request to send adsb data to vehicle as it is received

    return true;
}


void ADSB_Sagetech_MXS::update_serial_output(const Aircraft *sitl_model)
{
    // TODO: space checks here

    if (ack_info.send) {
        uint8_t alt[3] {};
        uint8_t state = 0;
        Ack ack{
            ack_info.ackType,
            ack_info.ackId,
            state,  // bitmask field
            alt
        };
        const PackedMessage<Ack> packed { ack, MsgType::ACK, msgid++ };
        write_to_autopilot((const char*)&packed, sizeof(packed));
        ack_info.send = false;
    }

    update_serial_output_vehicles(sitl_model);
}


void ADSB_Sagetech_MXS::update_serial_output_vehicles(const SITL::Aircraft *sitl_model)
{
    if (sitl_model->adsb == nullptr) {
        return;
    }
    for (uint8_t i=0; i<sitl_model->adsb->num_vehicles; i++) {
        const ADSB_Vehicle &vehicle = sitl_model->adsb->vehicles[i];
        if (!vehicle.initialised) {
            continue;
        }
        send_vehicle_message(vehicle);
    }
}

// pack a floating-point latitude or longitude into three bytes
// according to Sagetech definitions:
void ADSB_Sagetech_MXS::pack_scaled_geocoord(uint8_t buf[3], float coord)
{
    const int32_t scaled = coord * (1U<<23) / 180.0;
    pack_int32_into_uint8_ts(scaled, buf);
}

// pack a floating-point altitude in metres into three bytes
// according to Sagetech definitions:
void ADSB_Sagetech_MXS::pack_scaled_alt(uint8_t buf[3], float alt_m)
{
    const int32_t scaled = METRES_TO_FEET*alt_m * (1/0.015625);
    pack_int32_into_uint8_ts(scaled, buf);
}

// discards the top 8 bits from source and shoves the rest into dest
void ADSB_Sagetech_MXS::pack_int32_into_uint8_ts(int32_t source, uint8_t dest[3])
{
    // FIXME: cast &source instead?
    dest[0] = source >> 16;
    dest[1] = source >> 8;
    dest[2] = source >> 0;
}

uint8_t ADSB_Sagetech_MXS::scaled_groundspeed(float speed_m_s) const
{
    if (is_zero(speed_m_s)) {
        return 0x01;
    }
    const float knots = M_PER_SEC_TO_KNOTS * speed_m_s;
    if (knots < 0.125) {
        return 0x02;
    }
    static const struct Threshold {
        float min;
        uint8_t code;
        float increment;
    } thresholds[] {
        {   0.125, 0x03, 0.146 },
        {   1,     0x09, 0.25 },
        {   2,     0x0D, 0.50 },
        {  15,     0x27, 1 },
        {  70,     0x5e, 2 },
        { 100,     0x6d, 5 },
        { 175,     0x7c, 0 },
    };
    auto *entry = &thresholds[0];
    for (uint8_t i=1; i<ARRAY_SIZE(thresholds); i++) {
        auto *next_entry = &thresholds[i];
        if (knots > entry->min && knots < next_entry->min) {
            const uint8_t code_delta = next_entry->code - entry->code;
            return entry->code + uint8_t((knots - entry->min) / code_delta);
        }
        entry = next_entry;
    }
    return 0x7c;
}
void ADSB_Sagetech_MXS::pack_scaled_groundspeed(uint8_t dest[1], float speed_m_s) const
{
    dest[0] = scaled_groundspeed(speed_m_s);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "speed in: %f", speed_m_s);
}

void ADSB_Sagetech_MXS::pack_scaled_airspeed(uint8_t dest[2], float speed_m_s) const
{
    const int16_t scaled = M_PER_SEC_TO_KNOTS * speed_m_s * 8;
    dest[0] = scaled >> 8;
    dest[1] = scaled >> 0;
}

void ADSB_Sagetech_MXS::pack_scaled_vertical_rate(uint8_t dest[2], float speed_m_s) const
{
    const int16_t scaled = METRES_TO_FEET * speed_m_s;
    dest[0] = scaled >> 8;
    dest[1] = scaled >> 0;
}

void ADSB_Sagetech_MXS::send_vehicle_message(const SITL::ADSB_Vehicle &vehicle)
{
    send_vehicle_message_state_vector(vehicle);
    send_vehicle_message_status_report(vehicle);
}

void ADSB_Sagetech_MXS::send_vehicle_message_state_vector(const SITL::ADSB_Vehicle &vehicle)
{
    enum class SVR_ValidityFlag : uint8_t {
        LatitudeAndLongitude =      (1U << 7),
        Altitude_Geometric =        (1U << 6),
        NS_and_EW_Velocity =        (1U << 5),
        GroundSpeedWhileOnSurface = (1U << 4),
        HeadingWhileOnSurface     = (1U << 3),
        Altitude_Barometric       = (1U << 2),
        VerticalRate_Geometric    = (1U << 1),
        VerticalRate_Barometric   = (1U << 0),
    };
    enum class SVR_EstimateValidityFlag : uint8_t {
        LatitudeAndLongitude =      (1U << 7),
        NS_and_EW_Velocity =        (1U << 6),
    };

    // this struct-based approach may prove to be too inflexible
    // if we want to change the shape of the report at runtime for
    // some reason.
    struct PACKED {
        StateVectorReport_ReportStructure report_structure;
        uint8_t validity_flags;  // see SVR_ValidityFlag enum
        uint8_t estimated_validity_flags;  // see SVR_EstimateValidityFlag enum
        uint8_t participant_address[3];
        uint8_t address_qualifier;
        uint16_t epos_toa;
        uint16_t pos_toa;
        uint16_t vel_toa;
        uint8_t latitude[3];
        uint8_t longitude[3];
        uint8_t alt_geometric[3];
        uint8_t ns_velocity[2];
        uint8_t ew_velocity[2];
        uint8_t up_velocity[2];
        // uint8_t groundspeed;
        // uint8_t ground_heading;
    } my_report {};

    my_report.report_structure.rs0.msn = 1;

    // note that the Sagetech SDK parser requires all of these fields
    // to be 1 to be an airborne vehicle::
    my_report.report_structure.rs0.time_of_applicability_for_estimated_positon = 1;
    my_report.report_structure.rs0.position_time_of_applicability = 1;
    my_report.report_structure.rs0.velocity_time_of_applicability = 1;

    my_report.report_structure.rs2.estimated_longitude = 0;

    my_report.validity_flags = 0;

    pack_int32_into_uint8_ts(vehicle.ICAO_address, my_report.participant_address);
    my_report.address_qualifier = 0x02;  // aircraft

    Location loc = vehicle.get_location();

    // pack in latitude/longitude:
    pack_scaled_geocoord(my_report.latitude, loc.lat * 1e-7);
    pack_scaled_geocoord(my_report.longitude, loc.lng * 1e-7);
    my_report.report_structure.rs0.lat_and_lng = 1;
    my_report.validity_flags |= (uint8_t)SVR_ValidityFlag::LatitudeAndLongitude;

    // pack in altitude:
    pack_scaled_alt(my_report.alt_geometric, loc.alt*0.01);
    my_report.report_structure.rs1.altitude_geometric = 1;
    my_report.validity_flags |= (uint8_t)SVR_ValidityFlag::Altitude_Geometric;

    // pack in north/south and east/west velocity
    pack_scaled_airspeed(my_report.ns_velocity, vehicle.velocity_ef.x);
    pack_scaled_airspeed(my_report.ew_velocity, vehicle.velocity_ef.y);
    my_report.report_structure.rs1.north_south_east_west_vel = 1;
    my_report.validity_flags |= (uint8_t)SVR_ValidityFlag::NS_and_EW_Velocity;

    // pack in surface speed and heading:
    // pack_scaled_groundspeed(&my_report.groundspeed, vehicle.velocity_ef.length());
    // my_report.report_structure.rs1.ground_speed_on_ground = 1;
    // my_report.validity_flags |= (uint8_t)SVR_ValidityFlag::GroundSpeedWhileOnSurface;
    // my_report.report_structure.rs1.heading_on_ground = 1;
    // my_report.ground_heading = wrap_180(vehicle.velocity_ef.xy().angle()) / 1.40625;
    // my_report.validity_flags |= (uint8_t)SVR_ValidityFlag::HeadingWhileOnSurface;

    // pack in vertical rate
    pack_scaled_vertical_rate(my_report.up_velocity, -vehicle.velocity_ef.z);
    my_report.report_structure.rs1.vertical_rate_geometric_barometric = 1;
    my_report.validity_flags |= (uint8_t)SVR_ValidityFlag::VerticalRate_Geometric;

    StateVectorReportBuffer<sizeof(my_report)> report;
    memcpy(report.buffer, &my_report, sizeof(report.buffer));

    const PackedMessage<StateVectorReportBuffer<sizeof(my_report)>> packed {
        report,
        MsgType::STATEVECTORREPORT,
        msgid++
    };
    write_to_autopilot((const char*)&packed, sizeof(packed));
}

void ADSB_Sagetech_MXS::send_vehicle_message_status_report(const SITL::ADSB_Vehicle &vehicle)
{
    enum class RS0 : uint8_t {
        TimeOfApplicability       = (1U << 3),
        ADSB_Version              = (1U << 2),
        CallSign                  = (1U << 1),
        EmitterCategory           = (1U << 0),
    };
    enum class RS1 : uint8_t {
        AVLengthAndWidthCode    = (1U << 7),
        EmergencyPriorityStatus = (1U << 6),
        CapabilityCodes         = (1U << 5),
        OperationalMode         = (1U << 4),
        SVQualityNACp           = (1U << 3),
        SVQualityNACv           = (1U << 2),
        SVQualitySIL            = (1U << 1),
        SQQualtity_GVA          = (1U << 0),
    };
    enum class RS2 : uint8_t {
        SVQualityNICbaro        = (1U << 7),
        TrueMagneticHeading     = (1U << 6),
        VerticalRateType        = (1U << 5),
        FlightModeSpecific      = (1U << 4),
        Reserved3               = (1U << 3),
        Reserved2               = (1U << 2),
        Reserved1               = (1U << 1),
        Reserved0               = (1U << 0),
    };

    // this struct-based approach may prove to be too inflexible
    // if we want to change the shape of the report at runtime for
    // some reason.
    struct PACKED {
        uint32_t report_structure : 24;
        uint8_t validity_flags;  // see SVR_ValidityFlag enum
        uint8_t participant_address[3];
        uint8_t address_qualifier;
        uint8_t callsign[8];
    } my_report {};

    my_report.report_structure = uint8_t(RS0::CallSign) | (0x2 << 4);
    my_report.validity_flags = 0;

    pack_int32_into_uint8_ts(vehicle.ICAO_address, my_report.participant_address);
    my_report.address_qualifier = 0x02;  // aircraft

    memset(my_report.callsign, '\0', ARRAY_SIZE(my_report.callsign));
    memcpy(my_report.callsign, vehicle.callsign, MIN(ARRAY_SIZE(vehicle.callsign), ARRAY_SIZE(my_report.callsign)));

    ModeStatusReportBuffer<sizeof(my_report)> report;
    memcpy(report.buffer, &my_report, sizeof(report.buffer));

    const PackedMessage<ModeStatusReportBuffer<sizeof(my_report)>> packed {
        report,
        MsgType::MODESTATUSREPORT,
        msgid++
    };
    write_to_autopilot((const char*)&packed, sizeof(packed));
}

void ADSB_Sagetech_MXS::update_rf_input()
{
    // could selectivly copy from sitl->adsb here
}

void ADSB_Sagetech_MXS::update_rf_output()
{
    // print a message here when we are transmitting
}

void ADSB_Sagetech_MXS::update(const SITL::Aircraft *sitl_model)
{
    // see if we should do a report.
    if ((AP::sitl()->adsb_types & (1U << (uint8_t)SIM::ADSBType::SageTechMXS)) == 0) {
        return;
    }

    update_serial_input();
    update_serial_output(sitl_model);

    update_rf_input();
    update_rf_output();
}

#endif  // AP_SIM_ADSB_SAGETECH_MXS_ENABLED
