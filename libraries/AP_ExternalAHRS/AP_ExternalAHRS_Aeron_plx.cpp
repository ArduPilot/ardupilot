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

   ATH- Vinayak Pundhir (VP)
 */

/*
  Aeron Systems PLX series INS: External AHRS backend.
  See AP_ExternalAHRS_Aeron_plx.h for protocol overview and usage.
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_AERON_PLX_ENABLED

#include "AP_ExternalAHRS_Aeron_plx.h"

#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

// Local debug macro - no-op in release builds.
#ifndef AERON_DEBUG
#define AERON_DEBUG 0
#endif
#if AERON_DEBUG
#define debug(fmt, args ...) GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Aeron: " fmt, ##args)
#else
#define debug(...) do {} while (0)
#endif

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_Aeron_plx::AP_ExternalAHRS_Aeron_plx(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state)
    : AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart     = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Aeron: Serial port not found");
        return;
    }

    // Not offering IMU by default; rates below 400 Hz are too slow for
    // multicopter platforms. Users running SENS_PARA at >= 400 Hz can
    // opt into IMU via EAHRS_SENSORS.
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_Aeron_plx::update_thread, void),
            "Aeron",
            2048,
            AP_HAL::Scheduler::PRIORITY_SPI,
            0)) {
        AP_HAL::panic("Aeron: failed to create update thread");
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "Aeron: Initialised on SERIAL%d @ %lu baud",
                  port_num,
                  (unsigned long)baudrate);
}

void AP_ExternalAHRS_Aeron_plx::update_thread()
{
    uart->begin(baudrate, 2048, 256);
    setup_complete = true;

    while (true) {
        check_and_decode();
        report_hw_status();
        hal.scheduler->delay_microseconds(1000);
    }
}

// Core pipeline: read -> parse -> publish -> deferred messages.
void AP_ExternalAHRS_Aeron_plx::check_and_decode()
{
    if (uart == nullptr) {
        return;
    }
    
    // Drain buffer, stack-local. 256 B is ~3x the max bytes that arrive
    // in one 1 ms tick at 921600 baud (~92 B), and still clears a full
    // UART RX backlog within a few ticks if the thread is briefly
    // preempted.
    uint8_t chunk_buf[256];
    const uint16_t avail = MIN(uart->available(), uint16_t(sizeof(chunk_buf)));
    if (avail == 0) {
        return;
    }

    const auto n_read = uart->read(chunk_buf, avail);

    if (n_read <= 0) {
        return;
    }

    // msgs collects GCS notifications raised mid-parse (CRC failures,
    // unknown packets, ext-sensor replies) so they are emitted once per
    // drain instead of from inside the byte loop.
    AeronDeferredMsgs msgs{};

    for (auto index = 0; index < n_read; index++) {
        parse_byte(chunk_buf[index], msgs);
    }

    send_deferred_messages(msgs);

    check_for_query_ext_sensors();
}

// State-machine parser for the Aeron binary protocol.  On a CRC-valid
// frame, dispatches via handle_chunk_buf_packet() rather than leaving
// the caller to read implicitly-set member state.  CRC failures and
// unknown packets are flagged in msgs for the caller to surface as
// throttled GCS warnings.
void AP_ExternalAHRS_Aeron_plx::parse_byte(uint8_t byte, AeronDeferredMsgs &msgs)
{
    switch (parse_state) {
    case ParseState::RESET:
        write_idx   = 0;
        pkt_length  = 0;
        sync_count  = 0;
        parse_state = ParseState::SYNC;
        FALLTHROUGH;

    case ParseState::SYNC:
        if (byte == SYNC_BYTE) {
            rx_buf[write_idx++] = byte;
            if (++sync_count == SYNC_REQUIRED) {
                parse_state = ParseState::LEN_HIGH;
            }
        } else {
            parse_state = ParseState::RESET;
        }
        return;

    case ParseState::LEN_HIGH:
        pkt_length          = byte;
        rx_buf[write_idx++] = byte;
        parse_state         = ParseState::LEN_LOW;
        return;

    case ParseState::LEN_LOW:
        rx_buf[write_idx++] = byte;
        pkt_length = (pkt_length << 8) | byte;
        if (pkt_length >= PKT_LEN_MIN && pkt_length <= PKT_LEN_MAX) {
            parse_state = ParseState::PAYLOAD;
        } else {
            parse_state = ParseState::RESET;
        }
        return;

    case ParseState::PAYLOAD:
        rx_buf[write_idx++] = byte;
        if (write_idx >= pkt_length) {
            parse_state = ParseState::CRC_BYTE;
        }
        return;

    case ParseState::CRC_BYTE: {
        rx_buf[write_idx++] = byte;
        const bool valid = (byte == crc_xor_of_bytes(rx_buf, pkt_length));
        if (valid) {
            const uint16_t prp_id      = be16toh_ptr(&rx_buf[6]);
            const uint16_t payload_len = (pkt_length > 8) ? (pkt_length - 8) : 0;
            handle_chunk_buf_packet(prp_id, &rx_buf[8], payload_len, msgs);
        } else {
            msgs.crc_fail = true;
            debug("CRC-ID=0x%04X",
                  (uint16_t(rx_buf[6]) << 8) | rx_buf[7]);
        }
        parse_state = ParseState::RESET;
        return;
    }
    }

    // Unreachable - all enum values handled above.
    parse_state = ParseState::RESET;
}

// Dispatch a CRC-valid frame.  The frame sits in rx_buf[0 .. pkt_length-1];
// PRP_ID is at offset 6..7 (big-endian) and the payload starts at offset 8.
// Called from inside parse_byte's CRC state so the data hasn't yet been
// overwritten by the next incoming byte.
void AP_ExternalAHRS_Aeron_plx::handle_chunk_buf_packet(uint16_t prp_id,
                                                        const uint8_t *payload,
                                                        uint16_t payload_len,
                                                        AeronDeferredMsgs &msgs)
{
    switch (AeronPacketID(prp_id)) {
    case AeronPacketID::NAV_PARA1:
        if (payload_len < sizeof(NavPara1Payload)) {
            msgs.garbage = true;
            return;
        }
        publish_nav_para1(*reinterpret_cast<const NavPara1Payload *>(payload));
        return;

    case AeronPacketID::NAV_PARA2:
        if (payload_len < sizeof(NavPara2Payload)) {
            msgs.garbage = true;
            return;
        }
        publish_nav_para2(*reinterpret_cast<const NavPara2Payload *>(payload));
        return;

    case AeronPacketID::SENS_PARA:
        if (payload_len < sizeof(SensParaPayload)) {
            msgs.garbage = true;
            return;
        }
        publish_sens_para(*reinterpret_cast<const SensParaPayload *>(payload));
        return;

    case AeronPacketID::GPS_PARA:
        if (payload_len < sizeof(GpsParaPayload)) {
            msgs.garbage = true;
            return;
        }
        publish_gps_para(*reinterpret_cast<const GpsParaPayload *>(payload), msgs);
        return;

    case AeronPacketID::EXTD_GNSS:
        if (payload_len < sizeof(ExtdGnssPayload)) {
            msgs.garbage = true;
            return;
        }
        publish_extd_gnss(*reinterpret_cast<const ExtdGnssPayload *>(payload));
        return;

    case AeronPacketID::EXT_SENSORS_MAG: {
        if (payload_len < 1) {
            msgs.garbage = true;
            return;
        }
        WITH_SEMAPHORE(arn_sem);
        shared.ext_mag_present = payload[0];
        shared.ext_mag_checked = true;
        msgs.ext_mag_checked   = true;
        return;
    }

    case AeronPacketID::EXT_SENSORS_ASP: {
        if (payload_len < 1) {
            msgs.garbage = true;
            return;
        }
        WITH_SEMAPHORE(arn_sem);
        shared.ext_asp_present = payload[0];
        shared.ext_asp_checked = true;
        msgs.ext_asp_checked   = true;
        return;
    }

    case AeronPacketID::HEADING_S:
    case AeronPacketID::H_SPEED_S:
    case AeronPacketID::V_SPEED_S:
    case AeronPacketID::POSITION_S:
    case AeronPacketID::ALTITUDE_S:
    case AeronPacketID::DEV_INFO:
    case AeronPacketID::GNSS_PKT:
    case AeronPacketID::GNSS_FIX_STATUS:
        // Recognised but unused.
        return;
    }

    // Unknown packet ID - flag as garbage.
    msgs.garbage    = true;
    msgs.unknown_id = prp_id;
}

// NAV_PARA1: velocity, position, euler angles, course
void AP_ExternalAHRS_Aeron_plx::publish_nav_para1(const NavPara1Payload &data)
{
    last_nav1_ms = AP_HAL::millis();

    float undulation_m;
    {
        WITH_SEMAPHORE(arn_sem);
        memcpy(shared.nav1_pos_deg, data.position, sizeof(data.position));
        shared.nav1_height_m = data.height_abv_ellip;
        nav1_valid           = true;
        undulation_m         = shared.gps_undulation_m;
    }

    {
        WITH_SEMAPHORE(state.sem);
        state.velocity = {
            data.velocity_ned[0],
            data.velocity_ned[1],
            data.velocity_ned[2]
        };
        state.have_velocity = true;

        state.location = Location {
            int32_t(data.position[0] * 1.0e7),
            int32_t(data.position[1] * 1.0e7),
            int32_t((data.height_abv_ellip - undulation_m) * 1.0e2),
            Location::AltFrame::ABSOLUTE
        };
        state.have_location           = true;
        state.last_location_update_us = AP_HAL::micros();
    }

    // Logging is done outside the lock. We must not hold state.sem across that.
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_aern_log_ms >= LOG_THROTTLE_MS) {
        last_aern_log_ms = now_ms;
        // @LoggerMessage: AERN
        // @Description: Aeron PLX fused navigation solution (NAV_PARA1)
        // @Field: TimeUS: Time since system startup
        // @Field: Epoch: Unix epoch seconds at sample time
        // @Field: Lat: Latitude
        // @Field: Lng: Longitude
        // @Field: Alt: Height above WGS84 ellipsoid
        // @Field: VN: Velocity north
        // @Field: VE: Velocity east
        // @Field: VD: Velocity down
        // @Field: Crs: Course over ground
        // @Field: Roll: Roll angle
        // @Field: Pitch: Pitch angle
        // @Field: Yaw: Yaw angle
        // @Field: InsSt: PLX INS status word
        // @Field: HwSt: PLX hardware status word
        AP::logger().WriteStreaming(
            "AERN",
            "TimeUS," "Epoch," "Lat," "Lng," "Alt," "VN," "VE," "VD," "Crs," "Roll," "Pitch," "Yaw," "InsSt," "HwSt",
            "s"       "s"      "D"    "U"    "m"    "n"   "n"   "n"   "h"    "r"     "r"      "r"    "-"      "-",
            "F"       "-"      "-"    "-"    "-"    "-"   "-"   "-"   "-"    "-"     "-"      "-"    "-"      "-",
            "Q"       "I"      "d"    "d"    "f"    "f"   "f"   "f"   "f"    "f"     "f"      "f"    "I"      "I",
            AP_HAL::micros64(),
            data.epoch_time,
            data.position[0], data.position[1], double(data.height_abv_ellip),
            data.velocity_ned[0], data.velocity_ned[1], data.velocity_ned[2],
            data.course,
            data.euler[0], data.euler[1], data.euler[2],
            data.ins_status, 
            data.hw_status
        );
    }
#endif
}

// NAV_PARA2: quaternion, body velocity, ECEF, hw_status
void AP_ExternalAHRS_Aeron_plx::publish_nav_para2(const NavPara2Payload &data)
{
    last_nav2_ms = AP_HAL::millis();

    {
        WITH_SEMAPHORE(arn_sem);
        shared.nav2_hw_status = data.hw_status;
    }

    WITH_SEMAPHORE(state.sem);
    state.quat = {
        data.quat[0],
        data.quat[1],
        data.quat[2],
        data.quat[3]
    };
    state.have_quaternion = true;
}

// SENS_PARA: gyro/accel/mag/baro
void AP_ExternalAHRS_Aeron_plx::publish_sens_para(const SensParaPayload &data)
{
    last_sens_ms = AP_HAL::millis();

    const Vector3f accel {
        data.accel[0], 
        data.accel[1], 
        data.accel[2]
    };
    const Vector3f gyro {
        data.gyro[0] * DEG_TO_RAD,
        data.gyro[1] * DEG_TO_RAD,
        data.gyro[2] * DEG_TO_RAD
    };

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro  = gyro;
    }

    const AP_ExternalAHRS::ins_data_message_t ins {
        accel,
        gyro,
        data.temperature
    };
    AP::ins().handle_external(ins);

#if AP_BARO_EXTERNALAHRS_ENABLED
    const AP_ExternalAHRS::baro_data_message_t baro {
        0,                          // instance
        data.baro_pressure,
        data.baro_temperature
    };
    AP::baro().handle_external(baro);
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    const AP_ExternalAHRS::mag_data_message_t mag {
        Vector3f {
            data.mag[0],
            data.mag[1],
            data.mag[2]
        }
    };
    AP::compass().handle_external(mag);
#endif

#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_aers_log_ms >= LOG_THROTTLE_MS) {
        last_aers_log_ms = now_ms;
        // @LoggerMessage: AERS
        // @Description: Aeron PLX raw sensor data (SENS_PARA)
        // @Field: TimeUS: Time since system startup
        // @Field: Temp: Sensor temperature
        // @Field: GX: Gyro X
        // @Field: GY: Gyro Y
        // @Field: GZ: Gyro Z
        // @Field: AX: Accel X
        // @Field: AY: Accel Y
        // @Field: AZ: Accel Z
        // @Field: MX: Mag X
        // @Field: MY: Mag Y
        // @Field: MZ: Mag Z
        // @Field: Pres: Barometric pressure
        // @Field: BTmp: Barometer temperature
        // @Field: BAlt: Barometric altitude
        AP::logger().WriteStreaming(
            "AERS",
            "TimeUS," "Temp," "GX," "GY," "GZ," "AX," "AY," "AZ," "MX," "MY," "MZ," "Pres," "BTmp," "BAlt",
            "s"       "O"     "k"   "k"   "k"   "o"   "o"   "o"   "G"   "G"   "G"   "P"     "O"     "m",
            "F"       "-"     "-"   "-"   "-"   "-"   "-"   "-"   "-"   "-"   "-"   "-"     "-"     "-",
            "Q"       "f"     "f"   "f"   "f"   "f"   "f"   "f"   "f"   "f"   "f"   "f"     "f"     "f",
            AP_HAL::micros64(),
            data.temperature,
            data.gyro[0], data.gyro[1], data.gyro[2],
            data.accel[0], data.accel[1], data.accel[2],
            data.mag[0], data.mag[1], data.mag[2],
            data.baro_pressure, 
            data.baro_temperature, 
            data.baro_altitude
        );
    }
#endif
}

// GPS_PARA: GNSS status, position, DOP
void AP_ExternalAHRS_Aeron_plx::publish_gps_para(const GpsParaPayload &data, AeronDeferredMsgs &msgs)
{
    last_gnss_ms = AP_HAL::millis();

    bool have_nav1;
    {
        WITH_SEMAPHORE(arn_sem);
        shared.gps_epoch_s      = data.epoch_time;
        shared.gps_microseconds = data.microseconds;
        shared.gps_status       = data.gps_status;
        shared.gps_undulation_m = data.gps_undulation;
        shared.gps_hdop         = data.hdop;
        have_nav1               = nav1_valid;
    }

    // Build the AP_GPS message only after at least one NAV_PARA1 has
    // populated shared.nav1_pos_deg / shared.nav1_height_m.
    if (have_nav1) {
        format_and_push_gps(msgs);
    }
}

// EXTD_GNSS: accuracy metrics + GNSS NED velocity
void AP_ExternalAHRS_Aeron_plx::publish_extd_gnss(const ExtdGnssPayload &data)
{
    WITH_SEMAPHORE(arn_sem);

    shared.cust_hpa_m   = data.hpa;
    shared.cust_vpa_m   = data.vpa;
    shared.cust_hva_mps = data.hva;
    shared.cust_vdop    = data.vdop;
    memcpy(shared.cust_gnss_vel_ned_mps, data.gnss_vned, sizeof(data.gnss_vned));

    last_extd_ms      = AP_HAL::millis();
    has_variance_data = true;
}

/*
  Build the ArduPilot GPS message from GPS_PARA + EXTD_GNSS fields.
 */
void AP_ExternalAHRS_Aeron_plx::format_and_push_gps(AeronDeferredMsgs &msgs)
{
    // One coherent snapshot of the cross-thread cache, taken under
    // arn_sem and released immediately. Everything below works off the
    // local copy, so no shared.* field is read while unlocked.
    SharedData snap;
    bool extd_stale;
    {
        WITH_SEMAPHORE(arn_sem);
        snap       = shared;
        extd_stale = (last_extd_ms == 0) ||
                     (AP_HAL::millis() - last_extd_ms > EXTD_VALIDITY_MS);
    }

    const uint32_t st          = snap.gps_status;
    const uint16_t num_sats    = st & 0x000003FFU;
    const uint8_t  fix_raw     = (st & 0x00007800U) >> 11;
    const uint8_t  jam         = (st & 0x00030000U) >> 16;
    const uint8_t  spf         = (st & 0x000C0000U) >> 18;

    // throttled jamming/spoofing warnings - 0.2 Hz
    const uint32_t now = AP_HAL::millis();
    if (now - last_jam_warn_ms > WARN_THROTTLE_MS) {
        last_jam_warn_ms = now;
        if (jam >= uint8_t(AeronJamSpoof::WARNING)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Aeron: GNSS jamming level %u", jam);
        }
        if (spf >= uint8_t(AeronJamSpoof::WARNING)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Aeron: GNSS spoofing level %u", spf);
        }
    }

    AP_ExternalAHRS::gps_data_message_t gps_msg{};

    // Convert the PLX POSIX epoch to GPS week / ms-of-week using
    // AP_GPS.h own constants. UNIX_OFFSET_MSEC already
    // folds in the leap-second offset, so no separate leap term is added
    // here - and the leap count tracks AP_GPS.h global value rather
    // than a private copy.
    const uint64_t epoch_ms = uint64_t(snap.gps_epoch_s) * 1000ULL +
                              (snap.gps_microseconds / 1000ULL);
    if (epoch_ms >= UNIX_OFFSET_MSEC) {
        const uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
        gps_msg.gps_week = uint16_t(gps_ms / AP_MSEC_PER_WEEK);
        gps_msg.ms_tow   = uint32_t(gps_ms % AP_MSEC_PER_WEEK);
    }

    // satellites_in_view is uint8_t; the PLX field is 10-bit.
    gps_msg.satellites_in_view = MIN(num_sats, uint16_t(255));

    // PLX fix type -> ArduPilot fix type. We have received a GPS_PARA
    // packet, so the receiver is present; num_sats == 0 or an unrecognised 
    // fix code is reported as NONE (receiver alive, no position fix).
    if (num_sats > 0) {
        switch (AeronGnssFix(fix_raw)) {
        case AeronGnssFix::NO_FIX:
            gps_msg.fix_type = AP_GPS_FixType::NONE;
            break;
        case AeronGnssFix::GNSS_FIX:
            gps_msg.fix_type = AP_GPS_FixType::FIX_3D;
            break;
        case AeronGnssFix::SBAS_FIX:
            gps_msg.fix_type = AP_GPS_FixType::DGPS;
            break;
        case AeronGnssFix::RTK_FIX:
            gps_msg.fix_type = AP_GPS_FixType::RTK_FIXED;
            break;
        case AeronGnssFix::RTK_FLOAT:
            gps_msg.fix_type = AP_GPS_FixType::RTK_FLOAT;
            break;
        default:
            // Unrecognised PLX fix - alive but untrusted position.
            gps_msg.fix_type = AP_GPS_FixType::NONE;
            break;
        }
    } else {
        gps_msg.fix_type = AP_GPS_FixType::NONE;
    }

    // accuracy metrics from EXTD_GNSS (fresh - checked above)
    gps_msg.horizontal_pos_accuracy = snap.cust_hpa_m;
    gps_msg.vertical_pos_accuracy   = snap.cust_vpa_m;
    gps_msg.horizontal_vel_accuracy = snap.cust_hva_mps;
    gps_msg.hdop = snap.gps_hdop  * 1.0e2;
    gps_msg.vdop = snap.cust_vdop * 1.0e2;

    // position from NAV_PARA1 (fused INS/GNSS), kept in double end-to-end
    gps_msg.latitude     = snap.nav1_pos_deg[0] * 1.0e7;
    gps_msg.longitude    = snap.nav1_pos_deg[1] * 1.0e7;
    gps_msg.msl_altitude = (snap.nav1_height_m - snap.gps_undulation_m) * 1.0e2;

    // GNSS velocity from EXTD_GNSS (fresh)
    gps_msg.ned_vel_north = snap.cust_gnss_vel_ned_mps[0];
    gps_msg.ned_vel_east  = snap.cust_gnss_vel_ned_mps[1];
    gps_msg.ned_vel_down  = snap.cust_gnss_vel_ned_mps[2];

    // Set origin on the first 3D fix. The PLX only declares a 3D fix
    // after its own internal quality checks, so no extra gating here.
    if (gps_msg.fix_type >= AP_GPS_FixType::FIX_3D) {
        WITH_SEMAPHORE(state.sem);
        if (!state.have_origin) {
            state.origin = Location {
                int32_t(snap.nav1_pos_deg[0] * 1.0e7),
                int32_t(snap.nav1_pos_deg[1] * 1.0e7),
                int32_t((snap.nav1_height_m - snap.gps_undulation_m) * 1.0e2),
                Location::AltFrame::ABSOLUTE
            };
            state.have_origin = true;
        }
    }

    // If EXTD_GNSS is stale we have no fresh accuracy or GNSS velocity to
    // attach. Rather than publish a GPS message with fabricated accuracy
    // and zeroed velocity - which raw-data consumers (DCM, vehicle code)
    // would read as real - we skip publication entirely and let AP_GPS
    // time out naturally, which every consumer handles. The
    // fused INS solution still reaches ArduPilot via state.* (NAV_PARA1).
    // This will happen in very rare conditions, where device itself is compromised,
    // as it's hard configured to send these data.
    if (extd_stale) {
        // Shout on telemetry that rare condition has occurred.
        msgs.stale_data = true;
        return;
    }

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps_msg, instance);
    }

#if HAL_LOGGING_ENABLED
    // AERG follows the GNSS rate (5-10 Hz) so it is logged per-packet.
    // @LoggerMessage: AERG
    // @Description: Aeron PLX GNSS status and accuracy (GPS_PARA + EXTD_GNSS)
    // @Field: TimeUS: Time since system startup
    // @Field: GMS: Time of week
    // @Field: GWk: GPS week
    // @Field: NSat: Satellites in view
    // @Field: FixT: GPS fix type as reported to AP_GPS
    // @Field: Jam: Jamming detection level (1=none,2=warn,3=critical)
    // @Field: Spf: Spoofing detection level (1=none,2=warn,3=critical)
    // @Field: HPA: Horizontal position accuracy
    // @Field: VPA: Vertical position accuracy
    // @Field: HVA: Horizontal velocity accuracy
    // @Field: HDOP: Horizontal dilution of precision
    // @Field: VDOP: Vertical dilution of precision
    AP::logger().WriteStreaming(
        "AERG",
        "TimeUS," "GMS," "GWk," "NSat," "FixT," "Jam," "Spf," "HPA," "VPA," "HVA," "HDOP," "VDOP",
        "s"       "-"    "-"    "-"     "-"     "-"    "-"    "-"    "-"    "-"    "-"     "-",
        "F"       "-"    "-"    "-"     "-"     "-"    "-"    "-"    "-"    "-"    "-"     "-",
        "Q"       "I"    "H"    "H"     "B"     "B"    "B"    "f"    "f"    "f"    "f"     "f",
        AP_HAL::micros64(),
        gps_msg.ms_tow,
        gps_msg.gps_week,
        num_sats,
        uint8_t(gps_msg.fix_type),
        jam,
        spf,
        snap.cust_hpa_m,
        snap.cust_vpa_m,
        snap.cust_hva_mps,
        snap.gps_hdop,
        snap.cust_vdop
    );
#endif
}

void AP_ExternalAHRS_Aeron_plx::send_deferred_messages(const AeronDeferredMsgs &msgs)
{
    const uint32_t now = AP_HAL::millis();

    if (msgs.crc_fail && (now - last_crc_warn_ms) >= WARN_THROTTLE_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Aeron: CRC mismatch");
        last_crc_warn_ms = now;
    }

    if (msgs.garbage && (now - last_garbage_warn_ms) >= WARN_THROTTLE_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Aeron: Unknown packet 0x%04X", msgs.unknown_id);
        last_garbage_warn_ms = now;
    }

    if (msgs.stale_data && (now - last_stale_data_ms) >= WARN_THROTTLE_MS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Aeron: EXTD_GNSS Stale data, check packet!");
        last_stale_data_ms = now;
    }

    if (msgs.ext_mag_checked) {
        uint8_t present;
        {
            WITH_SEMAPHORE(arn_sem);
            present = shared.ext_mag_present;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Ext mag present=%u", present);
    }
    if (msgs.ext_asp_checked) {
        uint8_t present;
        {
            WITH_SEMAPHORE(arn_sem);
            present = shared.ext_asp_present;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Ext airspeed present=%u", present);
    }
}

// Fire the external-sensor presence queries once NAV data confirms the
// PLX is up and producing a fused solution. If NAV data lapses (a device
// reset or link dropout) and then resumes, re-query so the presence flags
// reflect the re-initialised device rather than the pre-reset state.
void AP_ExternalAHRS_Aeron_plx::check_for_query_ext_sensors()
{
    // Either NAV packet proves the device is producing a solution; use
    // whichever arrived most recently as the "device up" timestamp.
    const uint32_t nav_ms = MAX(last_nav1_ms, last_nav2_ms);

    // nav_ms == 0 means either no NAV yet, or the timestamp landed exactly
    // on the millis() rollover (every ~50 days). Both are benign: we
    // just skip this cycle and resume on the next NAV packet (~20 ms
    // later). The only cost is at most one deferred re-query check.
    if (nav_ms == 0) {
        return;
    }

    // A jump in the NAV timestamp larger than the reset threshold means
    // NAV stopped and later resumed - treat as a fresh boot and re-query.
    // (A reset landing exactly on the 50-day millis() wrap could be
    // missed here, but that coincidence is very unlikely and
    // the only consequence is not re-querying optional-sensor presence.)
    if (ext_sensor_query_done && prev_nav_ms != 0 &&
        (nav_ms - prev_nav_ms) > DEVICE_RESET_GAP_MS) {
        ext_sensor_query_done = false;
    }
    prev_nav_ms = nav_ms;

    if (!ext_sensor_query_done) {
        query_ext_sensors();
        ext_sensor_query_done = true;
    }
}

void AP_ExternalAHRS_Aeron_plx::query_ext_sensors()
{
    // External sensor presence queries - fixed packets sent once at startup
    static const uint8_t cmd_query_ext_mag[] = {
        0x05, 0x05, 0x05, 0x05, 0x00, 0x0A, 0x00, 0xA1, 0x01, 0x17, 0xBD
    };

    static const uint8_t cmd_query_ext_asp[] = {
        0x05, 0x05, 0x05, 0x05, 0x00, 0x0A, 0x00, 0xA1, 0x03, 0x62, 0xCA
    };

    uart->write(cmd_query_ext_mag, sizeof(cmd_query_ext_mag));
    uart->write(cmd_query_ext_asp, sizeof(cmd_query_ext_asp));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: External sensor query");
}

const char *AP_ExternalAHRS_Aeron_plx::get_name() const
{
    return "Aeron-PLX3";
}

int8_t AP_ExternalAHRS_Aeron_plx::get_port() const
{
    return uart ? port_num : -1;
}

bool AP_ExternalAHRS_Aeron_plx::initialised() const
{
    if (!setup_complete) {
        return false;
    }
    return last_sens_ms  != 0
        && last_nav1_ms  != 0
        && last_nav2_ms  != 0
        && last_gnss_ms  != 0;
}

// hw_status reporting table.  Order is not significant - report_hw_status
// walks every row to compute both the effective error mask and the
// fault / recovery messages.
const AP_ExternalAHRS_Aeron_plx::HwStatusEntry
AP_ExternalAHRS_Aeron_plx::hw_status_table[] = {
    { AeronHwStatus::GNSS,     MAV_SEVERITY_NOTICE,    "GNSS unhealthy",             "GNSS okay" },
    { AeronHwStatus::ACC,      MAV_SEVERITY_ERROR,     "Accelerometer unhealthy",    "Accel okay" },
    { AeronHwStatus::GYR,      MAV_SEVERITY_ERROR,     "Gyroscope unhealthy",        "Gyro okay" },
    { AeronHwStatus::MAG,      MAV_SEVERITY_ERROR,     "Magnetometer unhealthy",     "Mag okay" },
    { AeronHwStatus::BARO,     MAV_SEVERITY_ERROR,     "Barometer unhealthy",        "Baro okay" },
    { AeronHwStatus::SUP_VLTG, MAV_SEVERITY_WARNING,   "Supply Voltage Error",       "Supply voltage okay" },
    { AeronHwStatus::CM_PRT,   MAV_SEVERITY_WARNING,   "Comport Overrun",            "Comport okay" },
    { AeronHwStatus::RAM,      MAV_SEVERITY_EMERGENCY, "RAM Failure",                nullptr },
    { AeronHwStatus::FIRMWARE, MAV_SEVERITY_EMERGENCY, "Firmware Failure",           nullptr },
    { AeronHwStatus::CONFIG,   MAV_SEVERITY_EMERGENCY, "Config Memory Failure",      nullptr },
    { AeronHwStatus::EXT_MAG,  MAV_SEVERITY_WARNING,   "Ext Mag not connected",      "Ext Mag connected" },
    { AeronHwStatus::EXT_ASP,  MAV_SEVERITY_WARNING,   "Ext Airspeed not connected", "Ext Airspeed connected" },
};

/*
  Hardware-status reporting. Emits GCS warnings for any error flag in
  NAV_PARA2.hw_status, with rising-edge + per-flag-rate-limited
  semantics.

  Reporting strategy:
    - Rising edge (flag 0 -> 1):       report immediately, once.
    - Persistent fault (flag stays 1): repeat every HEALTH_REPEAT_INTERVAL_MS.
    - Falling edge (flag 1 -> 0):      log "okay" (if a clear_msg is set)
                                       and stop reporting.

  The hw_status flags do NOT fail healthy() - the PLX continues
  navigation with degraded sensor inputs. This routine only emits
  informational/warning GCS messages.
 */
void AP_ExternalAHRS_Aeron_plx::report_hw_status()
{
    const uint32_t now = AP_HAL::millis();

    // Snapshot the shared status fields under the lock so a publish
    // from the same thread (publish_nav_para2) can't tear us. Same
    // thread, but the lock is conventional protection that documents
    // the contract.
    uint32_t hw;
    uint8_t  ext_mag_present;
    uint8_t  ext_asp_present;
    bool     ext_mag_checked;
    bool     ext_asp_checked;
    {
        WITH_SEMAPHORE(arn_sem);
        hw              = shared.nav2_hw_status;
        ext_mag_present = shared.ext_mag_present;
        ext_asp_present = shared.ext_asp_present;
        ext_mag_checked = shared.ext_mag_checked;
        ext_asp_checked = shared.ext_asp_checked;
    }

    // Build the effective error mask from the table, then drop the
    // EXT_* flags unless the user actually has those sensors configured.
    uint32_t effective_mask = 0;
    for (const auto &entry : hw_status_table) {
        effective_mask |= 1U << uint8_t(entry.status);
    }
    if (!(ext_mag_checked && ext_mag_present)) {
        effective_mask &= ~(1U << uint8_t(AeronHwStatus::EXT_MAG));
    }
    if (!(ext_asp_checked && ext_asp_present)) {
        effective_mask &= ~(1U << uint8_t(AeronHwStatus::EXT_ASP));
    }

    const uint32_t errors      = hw & effective_mask;
    const uint32_t newly_set   = errors & ~last_warned_stat;
    const uint32_t newly_clear = last_warned_stat & ~errors;

    for (const auto &entry : hw_status_table) {
        const uint8_t  status_index = uint8_t(entry.status);
        const uint32_t mask         = 1U << status_index;

        // Recovery message on falling edge.
        if ((newly_clear & mask) != 0U && entry.clear_msg != nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: %s", entry.clear_msg);
        }

        if ((errors & mask) == 0U) {
            continue;
        }

        // Currently in fault: emit on rising edge or after the repeat
        // interval has elapsed.
        const bool rising  = (newly_set & mask) != 0U;
        const bool overdue = (now - last_warned_ms[status_index]) >= HEALTH_REPEAT_INTERVAL_MS;
        if (!rising && !overdue) {
            continue;
        }

        GCS_SEND_TEXT(MAV_SEVERITY(entry.severity), "Aeron: %s", entry.fault_msg);
        last_warned_ms[status_index] = now;
    }

    last_warned_stat = errors;
}

/*
  Health check. Pure stale-timestamp test.
 */
bool AP_ExternalAHRS_Aeron_plx::healthy() const
{
    if (!initialised()) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    // Both NAV streams are checked independently: NAV_PARA1 carries
    // position/velocity/attitude and NAV_PARA2 the quaternion + hw_status.
    // Losing either must trip unhealthy, otherwise stale data from the
    // lost stream would keep publishing at arbitrary age.
    return (now - last_sens_ms)  < SENS_TIMEOUT_MS
        && (now - last_nav1_ms)  < NAV_TIMEOUT_MS
        && (now - last_nav2_ms)  < NAV_TIMEOUT_MS
        && (now - last_gnss_ms)  < GNSS_TIMEOUT_MS;
}

/*
  Pre-arm check.

  GNSS-related warnings (no lock, jamming, spoofing) are surfaced via
  throttled GCS_SEND_TEXT in format_and_push_gps(), because the PLX
  can navigate in GNSS-denied mode and we want those warnings to be
  informational rather than blocking.
 */
bool AP_ExternalAHRS_Aeron_plx::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Aeron Not Initialised");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Aeron Unhealthy");
        return false;
    }
    return true;
}

/*
  Filter status.

  Reaching the body below means healthy() returned true, which already
  guarantees NAV_PARA1, NAV_PARA2, SENS and GNSS are all within their
  staleness deadlines. So the PLX's fused INS solution is currently
  valid, and every estimate it continuously produces - attitude, 3D
  velocity, and relative position (by inertial integration and aidings) - is
  available. Those flags are therefore set whenever healthy, not
  unconditionally.

  horiz_pos_abs / pred_horiz_pos_abs are the fused absolute position,
  which the INS holds through short GNSS outages by dead reckoning, so
  they follow the (healthy) NAV solution rather than the GNSS fix.
  using_gps / gps_quality_good track the raw GNSS fix bit specifically.
 */
void AP_ExternalAHRS_Aeron_plx::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    if (!initialised()) {
        return;
    }

    status.flags.initalized = true;

    if (!healthy()) {
        return;
    }

    bool fix;
    {
        WITH_SEMAPHORE(arn_sem);
        fix = (shared.gps_status & GPS_STATUS_FIX_BIT) != 0;
    }

    status.flags.attitude            = true;
    status.flags.horiz_vel           = true;
    status.flags.vert_vel            = true;
    status.flags.horiz_pos_rel       = true;
    status.flags.vert_pos            = true;
    status.flags.pred_horiz_pos_rel  = true;
    status.flags.using_gps           = fix;
    status.flags.gps_quality_good    = fix;

    // healthy() guarantees NAV is fresh, so the fused absolute position
    // is available now (held through GNSS outages by dead reckoning).
    status.flags.horiz_pos_abs      = true;
    status.flags.pred_horiz_pos_abs = true;
}

/*
  Variance estimates from EXTD_GNSS accuracy metrics. Returns false
  until the first EXTD_GNSS packet arrives so ArduPilot does not
  over-trust the solution during initialisation when accuracy fields
  are still zero.

  The PLX3-N INS does not expose EKF innovation variances, returning nothing
  leaves the report unpopulated. We therefore map the PLX3's own GNSS accuracy
  estimates onto variance fields as a best effort health signal.

  These are accuracy-derived values, not filter innovations.
 */
bool AP_ExternalAHRS_Aeron_plx::get_variances(float &velVar, float &posVar,
                                              float &hgtVar, Vector3f &magVar,
                                              float &tasVar) const
{
    float hpa;
    float vpa;
    float hva;
    {
        WITH_SEMAPHORE(arn_sem);

        if (!has_variance_data) {
            return false;
        }

        hpa = shared.cust_hpa_m;
        vpa = shared.cust_vpa_m;
        hva = shared.cust_hva_mps;
    }

    velVar = hva * vel_gate_scale;
    posVar = hpa * pos_gate_scale;
    hgtVar = vpa * hgt_gate_scale;
    magVar.zero();
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_AERON_PLX_ENABLED
