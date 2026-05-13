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
  See AP_ExternalAHRS_Aeron.h for protocol overview and usage.
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_AERON_PLX_ENABLED

#include "AP_ExternalAHRS_Aeron.h"

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

        // Periodic packet-rate diagnostics
        const uint32_t now = AP_HAL::millis();
        if (now - last_rate_ms >= RATE_LOG_INTERVAL_MS) {
            const uint32_t dt_s = RATE_LOG_INTERVAL_MS / 1000;
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                          "SENS %luHz NAV1 %luHz NAV2 %luHz GPS %luHz E.GPS %luHz",
                          (unsigned long)(sens_count / dt_s),
                          (unsigned long)(nav1_count / dt_s),
                          (unsigned long)(nav2_count / dt_s),
                          (unsigned long)(gps_count / dt_s),
                          (unsigned long)(extd_gnss_count / dt_s));

            sens_count = nav1_count = nav2_count = gps_count = extd_gnss_count = 0;
            last_rate_ms = now;
        }

        hal.scheduler->delay_microseconds(1000);
    }
}

// Core pipeline: read -> parse -> decode -> publish -> deferred messages.
// Returns true if any bytes were processed this call, false if no data
// was available.
bool AP_ExternalAHRS_Aeron_plx::check_and_decode()
{
    WITH_SEMAPHORE(arn_sem);

    if (uart == nullptr) {
        return false;
    }

    const uint16_t avail = MIN(uart->available(), uint16_t(sizeof(chunk_buf)));
    if (avail == 0) {
        return false;
    }

    const uint16_t n_read = uart->read(chunk_buf, avail);
    if (n_read == 0) {
        return false;
    }

    AeronDecNavPara1  nav1{};
    AeronDecNavPara2  nav2{};
    AeronDecSens      sens{};
    AeronDecGps       gps{};
    AeronDecCust      cust{};
    AeronDeferredMsgs msgs{};

    if (crc_fail_pending) {
        msgs.crc_fail     = true;
        crc_fail_pending  = false;
    }

    for (uint16_t index = 0; index < n_read; index++) {
        if (!parse_byte(chunk_buf[index])) {
            continue;
        }

        const uint16_t pkt_id = decode_to_local(&nav1, &nav2, &sens, &gps, &cust, &msgs);
        switch (pkt_id) {
        case uint16_t(AeronPacketID::NAV_PARA1):
            if (nav1.fresh) {
                publish_nav_para1(nav1);
                nav1.fresh = false;
                nav1_count++;
            }
            break;

        case uint16_t(AeronPacketID::NAV_PARA2):
            if (nav2.fresh) {
                publish_nav_para2(nav2);
                nav2.fresh = false;
                nav2_count++;
            }
            break;

        case uint16_t(AeronPacketID::SENS_PARA):
            if (sens.fresh) {
                publish_sens_para(sens);
                sens.fresh = false;
                sens_count++;
            }
            break;

        case uint16_t(AeronPacketID::GPS_PARA):
            if (gps.fresh) {
                publish_gps_para(gps);
                gps.fresh = false;
                gps_count++;
            }
            break;

        case uint16_t(AeronPacketID::EXTD_GNSS):
            if (cust.fresh) {
                publish_cust_pkt(cust);
                cust.fresh = false;
                extd_gnss_count++;
            }
            break;

        default:
            // Aiding-status acks, DEV_INFO and the EXT_SENSORS_*
            // responses are decoded inside decode_to_local() but don't
            // need an action here. Garbage / unknown IDs are reported
            // via the deferred message pipeline below.
            break;
        }
    }

    send_deferred_messages(msgs);

    if (!ext_sensor_query_done && AP_HAL::millis() > 10000) {
        query_ext_sensors();
        ext_sensor_query_done = true;
    }

    return true;
}

// State-machine parser for the Aeron binary protocol.
// Returns true only when a full, CRC-valid frame is complete.
bool AP_ExternalAHRS_Aeron_plx::parse_byte(uint8_t byte)
{
    switch (parse_state) {
    case ParseState::SYNC:
        if (byte == SYNC_BYTE) {
            rx_buf[write_idx++] = byte;
            if (++sync_count == SYNC_REQUIRED) {
                parse_state = ParseState::LEN_HIGH;
                sync_count  = 0;
            }
        } else {
            sync_count = 0;
            write_idx  = 0;
        }
        return false;

    case ParseState::LEN_HIGH:
        pkt_length = byte;
        rx_buf[write_idx++] = byte;
        parse_state = ParseState::LEN_LOW;
        return false;

    case ParseState::LEN_LOW:
        rx_buf[write_idx++] = byte;
        pkt_length = (pkt_length << 8) | byte;
        if (pkt_length >= PKT_LEN_MIN && pkt_length < PKT_LEN_MAX) {
            parse_state = ParseState::PAYLOAD;
        } else {
            parse_state = ParseState::SYNC;
            write_idx   = 0;
            pkt_length  = 0;
        }
        return false;

    case ParseState::PAYLOAD:
        rx_buf[write_idx++] = byte;
        if (write_idx >= pkt_length) {
            decoded_pkt_len = pkt_length;
            parse_state     = ParseState::CRC;
        }
        return false;

    case ParseState::CRC: {
        rx_buf[write_idx++] = byte;
        const bool valid = (byte == crc_xor_of_bytes(rx_buf, pkt_length));
        if (valid) {
            memcpy(decode_buf, rx_buf, pkt_length);
        } else {
            crc_fail_pending = true;
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                  "Aeron: CRC-ID=0x%04X",
                  (uint16_t(rx_buf[6]) << 8) | rx_buf[7]);
        }
        parse_state = ParseState::SYNC;
        write_idx   = 0;
        pkt_length  = 0;
        return valid;
    }
    }

    // Unreachable - all enum values handled above.
    parse_state = ParseState::SYNC;
    write_idx   = 0;
    pkt_length  = 0;
    return false;
}

/*
  Decode the framed packet from decode_buf into the appropriate local
  struct. No shared state is touched, except for the EXT_SENSORS_*
  cases which need to publish the presence flag.
 */
uint16_t AP_ExternalAHRS_Aeron_plx::decode_to_local(AeronDecNavPara1 *nav1,
                                                    AeronDecNavPara2 *nav2,
                                                    AeronDecSens *sens,
                                                    AeronDecGps *gps,
                                                    AeronDecCust *extd_gnss,
                                                    AeronDeferredMsgs *msgs)
{
    // payload starts after sync(4) + length(2) + prp_id(2)
    const uint8_t *p             = &decode_buf[8];
    const uint16_t prp_id        = be16toh_ptr(&decode_buf[6]);
    const uint16_t payload_len   = (decoded_pkt_len > 8) ? (decoded_pkt_len - 8) : 0;

    switch (prp_id) {
    case uint16_t(AeronPacketID::NAV_PARA1): {
        if (payload_len < uint16_t(AeronPacketLen::LEN_NAV_PARA1)) {
            msgs->garbage = true;
            return prp_id;
        }
        nav1->epoch_time      = le32toh_ptr(p +  0);
        nav1->microseconds    = le32toh_ptr(p +  4);
        nav1->ins_status      = le32toh_ptr(p +  8);
        nav1->hw_status       = le32toh_ptr(p + 12);
        nav1->euler[0]        = int32_to_float_le(le32toh_ptr(p + 16));
        nav1->euler[1]        = int32_to_float_le(le32toh_ptr(p + 20));
        nav1->euler[2]        = int32_to_float_le(le32toh_ptr(p + 24));
        nav1->course          = int32_to_float_le(le32toh_ptr(p + 28));
        nav1->velocity_ned[0] = int32_to_float_le(le32toh_ptr(p + 32));
        nav1->velocity_ned[1] = int32_to_float_le(le32toh_ptr(p + 36));
        nav1->velocity_ned[2] = int32_to_float_le(le32toh_ptr(p + 40));
        nav1->position[0]     = uint64_to_double_le(le64toh_ptr(p + 44));
        nav1->position[1]     = uint64_to_double_le(le64toh_ptr(p + 52));
        nav1->height_abv_ellip = int32_to_float_le(le32toh_ptr(p + 60));
        nav1->fresh = true;
        break;
    }

    case uint16_t(AeronPacketID::NAV_PARA2): {
        if (payload_len < uint16_t(AeronPacketLen::LEN_NAV_PARA2)) {
            msgs->garbage = true;
            return prp_id;
        }
        nav2->epoch_time   = le32toh_ptr(p +  0);
        nav2->microseconds = le32toh_ptr(p +  4);
        nav2->ins_status   = le32toh_ptr(p +  8);
        nav2->hw_status    = le32toh_ptr(p + 12);
        nav2->quat[0]      = int32_to_float_le(le32toh_ptr(p + 16));
        nav2->quat[1]      = int32_to_float_le(le32toh_ptr(p + 20));
        nav2->quat[2]      = int32_to_float_le(le32toh_ptr(p + 24));
        nav2->quat[3]      = int32_to_float_le(le32toh_ptr(p + 28));
        nav2->body_vel[0]  = int32_to_float_le(le32toh_ptr(p + 32));
        nav2->body_vel[1]  = int32_to_float_le(le32toh_ptr(p + 36));
        nav2->body_vel[2]  = int32_to_float_le(le32toh_ptr(p + 40));
        nav2->ecef_pos[0]  = uint64_to_double_le(le64toh_ptr(p + 44));
        nav2->ecef_pos[1]  = uint64_to_double_le(le64toh_ptr(p + 52));
        nav2->ecef_pos[2]  = uint64_to_double_le(le64toh_ptr(p + 60));
        nav2->altitude     = int32_to_float_le(le32toh_ptr(p + 68));
        nav2->fresh = true;
        break;
    }

    case uint16_t(AeronPacketID::SENS_PARA): {
        if (payload_len < uint16_t(AeronPacketLen::LEN_SENS_PARA)) {
            msgs->garbage = true;
            return prp_id;
        }
        sens->epoch_time       = le32toh_ptr(p +  0);
        sens->microseconds     = le32toh_ptr(p +  4);
        sens->temperature      = int32_to_float_le(le32toh_ptr(p +  8));
        sens->gyro[0]          = int32_to_float_le(le32toh_ptr(p + 12));
        sens->gyro[1]          = int32_to_float_le(le32toh_ptr(p + 16));
        sens->gyro[2]          = int32_to_float_le(le32toh_ptr(p + 20));
        sens->accel[0]         = int32_to_float_le(le32toh_ptr(p + 24));
        sens->accel[1]         = int32_to_float_le(le32toh_ptr(p + 28));
        sens->accel[2]         = int32_to_float_le(le32toh_ptr(p + 32));
        sens->mag[0]           = int32_to_float_le(le32toh_ptr(p + 36));
        sens->mag[1]           = int32_to_float_le(le32toh_ptr(p + 40));
        sens->mag[2]           = int32_to_float_le(le32toh_ptr(p + 44));
        sens->euler_rates[0]   = int32_to_float_le(le32toh_ptr(p + 48));
        sens->euler_rates[1]   = int32_to_float_le(le32toh_ptr(p + 52));
        sens->euler_rates[2]   = int32_to_float_le(le32toh_ptr(p + 56));
        sens->baro_pressure    = int32_to_float_le(le32toh_ptr(p + 60));
        sens->baro_temperature = int32_to_float_le(le32toh_ptr(p + 64));
        sens->baro_altitude    = int32_to_float_le(le32toh_ptr(p + 68));
        sens->fresh = true;
        break;
    }

    case uint16_t(AeronPacketID::GPS_PARA): {
        if (payload_len < uint16_t(AeronPacketLen::LEN_GPS_PARA)) {
            msgs->garbage = true;
            return prp_id;
        }
        gps->epoch_time           = le32toh_ptr(p +  0);
        gps->microseconds         = le32toh_ptr(p +  4);
        gps->gps_status           = le32toh_ptr(p +  8);
        gps->gps_position[0]      = uint64_to_double_le(le64toh_ptr(p + 12));
        gps->gps_position[1]      = uint64_to_double_le(le64toh_ptr(p + 20));
        gps->gps_height_abv_ellip = int32_to_float_le(le32toh_ptr(p + 28));
        gps->gps_undulation       = int32_to_float_le(le32toh_ptr(p + 32));
        gps->pdop                 = int32_to_float_le(le32toh_ptr(p + 36));
        gps->hdop                 = int32_to_float_le(le32toh_ptr(p + 40));
        gps->gdop                 = int32_to_float_le(le32toh_ptr(p + 44));
        gps->fresh = true;
        break;
    }

    case uint16_t(AeronPacketID::EXTD_GNSS): {
        if (payload_len < uint16_t(AeronPacketLen::LEN_EXTD_GNSS)) {
            msgs->garbage = true;
            return prp_id;
        }
        extd_gnss->hpa          = int32_to_float_le(le32toh_ptr(p +  0));
        extd_gnss->vpa          = int32_to_float_le(le32toh_ptr(p +  4));
        extd_gnss->hva          = int32_to_float_le(le32toh_ptr(p +  8));
        extd_gnss->vdop         = int32_to_float_le(le32toh_ptr(p + 12));
        extd_gnss->gnss_vned[0] = uint64_to_double_le(le64toh_ptr(p + 16));
        extd_gnss->gnss_vned[1] = uint64_to_double_le(le64toh_ptr(p + 24));
        extd_gnss->gnss_vned[2] = uint64_to_double_le(le64toh_ptr(p + 32));
        extd_gnss->fresh = true;
        break;
    }

    case uint16_t(AeronPacketID::EXT_SENSORS_MAG): {
        // Single-byte presence flag. Written under the lock so the
        // const main-thread reader in healthy() sees a consistent value.
        WITH_SEMAPHORE(state.sem);
        shared.ext_mag_present = p[0];
        shared.ext_mag_checked = true;
        msgs->ext_mag_checked  = true;
        break;
    }

    case uint16_t(AeronPacketID::EXT_SENSORS_ASP): {
        WITH_SEMAPHORE(state.sem);
        shared.ext_asp_present = p[0];
        shared.ext_asp_checked = true;
        msgs->ext_asp_checked  = true;
        break;
    }

    case uint16_t(AeronPacketID::HEADING_S):
    case uint16_t(AeronPacketID::H_SPEED_S):
    case uint16_t(AeronPacketID::V_SPEED_S):
    case uint16_t(AeronPacketID::POSITION_S):
    case uint16_t(AeronPacketID::ALTITUDE_S):
    case uint16_t(AeronPacketID::DEV_INFO):
        // Aiding-status acks and the DEV_INFO handshake reply.
        // We acknowledge their existence (the CRC-valid frame already
        // reset last_pkt_ms in parse_byte) but take no further action.
        break;

    case uint16_t(AeronPacketID::GNSS_PKT):
    case uint16_t(AeronPacketID::GNSS_FIX_STATUS):
        // Recognised but unused: detailed GNSS pseudorange / fix-status
        // packets that the PLX may emit if certain config bits are set.
        // Fused output already reaches us via NAV_PARA1 and GPS_PARA, so
        // these are intentionally ignored.
        break;

    default:
        msgs->garbage    = true;
        msgs->unknown_id = prp_id;
        break;
    }

    return prp_id;
}

// NAV_PARA1: velocity, position, euler angles, course
void AP_ExternalAHRS_Aeron_plx::publish_nav_para1(const AeronDecNavPara1 &data)
{
    last_nav_ms = AP_HAL::millis();

    {
        WITH_SEMAPHORE(state.sem);

        memcpy(shared.nav1_pos_deg, data.position, sizeof(data.position));
        shared.nav1_height_m = data.height_abv_ellip;

        state.velocity = Vector3f{data.velocity_ned[0],
                                  data.velocity_ned[1],
                                  data.velocity_ned[2]};
        state.have_velocity = true;

        state.location = Location{
            int32_t(data.position[0] * 1.0e7),
            int32_t(data.position[1] * 1.0e7),
            int32_t((data.height_abv_ellip - shared.gps_undulation_m) * 1.0e2),
            Location::AltFrame::ABSOLUTE};
        state.have_location           = true;
        state.last_location_update_us = AP_HAL::micros();

        nav1_valid = true;
    }

    // Logging is done outside the lock we must not hold state.sem across that.
#if HAL_LOGGING_ENABLED
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
    AP::logger().WriteStreaming("AERN", "TimeUS,Epoch,Lat,Lng,Alt,VN,VE,VD,Crs,Roll,Pitch,Yaw,InsSt,HwSt",
                                "ssDUmnnnhrrr--",
                                "F-------------",
                                "QIddffffffffII",
                                AP_HAL::micros64(),
                                data.epoch_time,
                                data.position[0], data.position[1], double(data.height_abv_ellip),
                                data.velocity_ned[0], data.velocity_ned[1], data.velocity_ned[2],
                                data.course,
                                data.euler[0], data.euler[1], data.euler[2],
                                data.ins_status, data.hw_status);
#endif
}

// NAV_PARA2: quaternion, body velocity, ECEF, hw_status
void AP_ExternalAHRS_Aeron_plx::publish_nav_para2(const AeronDecNavPara2 &data)
{
    last_nav_ms = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    shared.nav2_hw_status = data.hw_status;
    state.quat            = Quaternion{data.quat[0], data.quat[1],
                                       data.quat[2], data.quat[3]};
    state.have_quaternion = true;
}

// SENS_PARA: gyro/accel/mag/baro
void AP_ExternalAHRS_Aeron_plx::publish_sens_para(const AeronDecSens &data)
{
    last_sens_ms = AP_HAL::millis();

    const Vector3f accel{data.accel[0], data.accel[1], data.accel[2]};
    const Vector3f gyro{data.gyro[0] * DEG_TO_RAD,
                        data.gyro[1] * DEG_TO_RAD,
                        data.gyro[2] * DEG_TO_RAD};

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro  = gyro;
    }

    AP_ExternalAHRS::ins_data_message_t ins;
    ins.accel       = accel;
    ins.gyro        = gyro;
    ins.temperature = data.temperature;
    AP::ins().handle_external(ins);

#if AP_BARO_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance    = 0;
    baro.pressure_pa = data.baro_pressure;
    baro.temperature = data.baro_temperature;
    AP::baro().handle_external(baro);
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f{data.mag[0], data.mag[1], data.mag[2]};
    AP::compass().handle_external(mag);
#endif

#if HAL_LOGGING_ENABLED
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
    AP::logger().WriteStreaming("AERS", "TimeUS,Temp,GX,GY,GZ,AX,AY,AZ,MX,MY,MZ,Pres,BTmp,BAlt",
                                "sOkkkoooGGGPOm",
                                "F-------------",
                                "Qfffffffffffff",
                                AP_HAL::micros64(),
                                data.temperature,
                                data.gyro[0], data.gyro[1], data.gyro[2],
                                data.accel[0], data.accel[1], data.accel[2],
                                data.mag[0], data.mag[1], data.mag[2],
                                data.baro_pressure, data.baro_temperature, data.baro_altitude);
#endif
}

// GPS_PARA: GNSS status, position, DOP
void AP_ExternalAHRS_Aeron_plx::publish_gps_para(const AeronDecGps &data)
{
    last_gnss_ms = AP_HAL::millis();

    {
        WITH_SEMAPHORE(state.sem);
        shared.gps_epoch_s      = data.epoch_time;
        shared.gps_microseconds = data.microseconds;
        shared.gps_status       = data.gps_status;
        shared.gps_undulation_m = data.gps_undulation;
        shared.gps_hdop         = data.hdop;
    }

    // build the AP_GPS message only after at least one NAV_PARA1 has
    // populated shared.nav1_pos_deg / shared.nav1_height_m
    if (nav1_valid) {
        format_and_push_gps();
    }
}

// EXTD_GNSS: accuracy metrics + GNSS NED velocity
void AP_ExternalAHRS_Aeron_plx::publish_cust_pkt(const AeronDecCust &data)
{
    last_gnss_ms = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);

    shared.cust_hpa_m   = data.hpa;
    shared.cust_vpa_m   = data.vpa;
    shared.cust_hva_mps = data.hva;
    shared.cust_vdop    = data.vdop;
    memcpy(shared.cust_gnss_vel_ned_mps, data.gnss_vned, sizeof(data.gnss_vned));

    has_variance_data = true;
}

/*
  Build the ArduPilot GPS message from GPS_PARA + EXTD_GNSS fields.
 */
void AP_ExternalAHRS_Aeron_plx::format_and_push_gps()
{
    // extract GNSS status bitfields
    const uint32_t st          = shared.gps_status;
    const uint16_t num_sats    = st & 0x000003FFU;
    const bool     fix_present = (st & 0x00000400U) != 0;
    const uint8_t  fix_raw     = (st & 0x00007800U) >> 11;
    const uint8_t  jam         = (st & 0x00030000U) >> 16;
    const uint8_t  spf         = (st & 0x000C0000U) >> 18;

    gnss_fix     = fix_present;
    jam_status   = jam;
    spoof_status = spf;

    // throttled jamming/spoofing warnings - 0.2 Hz
    const uint32_t now = AP_HAL::millis();
    if (now - last_jam_warn_ms > 5000) {
        last_jam_warn_ms = now;
        if (jam >= uint8_t(AeronJamSpoof::WARNING)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Aeron: GNSS jamming level %u", jam);
        }
        if (spf >= uint8_t(AeronJamSpoof::WARNING)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Aeron: GNSS spoofing level %u", spf);
        }
    }

    AP_ExternalAHRS::gps_data_message_t gps_msg{};

    // Guard against epoch underflow if PLX has no time fix yet.
    if (shared.gps_epoch_s >= GPS_UNIX_OFFSET_S) {
        const uint32_t gps_time_s = shared.gps_epoch_s - GPS_UNIX_OFFSET_S;
        const uint32_t tow_s      = gps_time_s % SECONDS_PER_WEEK;
        gps_msg.gps_week = gps_time_s / SECONDS_PER_WEEK;
        gps_msg.ms_tow   = (tow_s * 1000) + (shared.gps_microseconds / 1000);
    } else {
        gps_msg.gps_week = 0;
        gps_msg.ms_tow   = 0;
    }

    gps_msg.satellites_in_view = num_sats;

    // PLX fix type -> ArduPilot fix type
    if (num_sats > 0) {
        switch (fix_raw) {
        case uint8_t(AeronGnssFix::NO_FIX):
            gps_msg.fix_type = AP_GPS_FixType::NONE;
            break;
        case uint8_t(AeronGnssFix::GNSS_FIX):
            gps_msg.fix_type = AP_GPS_FixType::FIX_3D;
            break;
        case uint8_t(AeronGnssFix::SBAS_FIX):
            gps_msg.fix_type = AP_GPS_FixType::DGPS;
            break;
        case uint8_t(AeronGnssFix::RTK_FIX):
            gps_msg.fix_type = AP_GPS_FixType::RTK_FIXED;
            break;
        case uint8_t(AeronGnssFix::RTK_FLOAT):
            gps_msg.fix_type = AP_GPS_FixType::RTK_FLOAT;
            break;
        default:
            // Reserved / unknown PLX fix code - report as no GPS so the
            // EKF doesn't trust the position. Should not happen in
            // practice; AeronGnssFix covers all values 0..4.
            gps_msg.fix_type = AP_GPS_FixType::NO_GPS;
            break;
        }
    } else {
        gps_msg.fix_type = AP_GPS_FixType::NO_GPS;
    }

    // accuracy metrics from EXTD_GNSS
    gps_msg.horizontal_pos_accuracy = shared.cust_hpa_m;
    gps_msg.vertical_pos_accuracy   = shared.cust_vpa_m;
    gps_msg.horizontal_vel_accuracy = shared.cust_hva_mps;
    gps_msg.hdop                    = shared.gps_hdop  * 1.0e2;
    gps_msg.vdop                    = shared.cust_vdop * 1.0e2;

    // position from NAV_PARA1
    gps_msg.latitude     = shared.nav1_pos_deg[0] * 1.0e7;
    gps_msg.longitude    = shared.nav1_pos_deg[1] * 1.0e7;
    gps_msg.msl_altitude = (shared.nav1_height_m - shared.gps_undulation_m) * 1.0e2;

    // GNSS velocity from EXTD_GNSS
    gps_msg.ned_vel_north = shared.cust_gnss_vel_ned_mps[0];
    gps_msg.ned_vel_east  = shared.cust_gnss_vel_ned_mps[1];
    gps_msg.ned_vel_down  = shared.cust_gnss_vel_ned_mps[2];

    // set origin on first valid 3D fix
    if (gps_msg.fix_type >= AP_GPS_FixType::FIX_3D) {
        WITH_SEMAPHORE(state.sem);
        if (!state.have_origin) {
            state.origin = Location {
                int32_t(shared.nav1_pos_deg[0] * 1.0e7),
                int32_t(shared.nav1_pos_deg[1] * 1.0e7),
                int32_t((shared.nav1_height_m - shared.gps_undulation_m) * 1.0e2),
                Location::AltFrame::ABSOLUTE };
            state.have_origin = true;
        }
    }

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps_msg, instance);
    }

#if HAL_LOGGING_ENABLED
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
    AP::logger().WriteStreaming("AERG", "TimeUS,GMS,GWk,NSat,FixT,Jam,Spf,HPA,VPA,HVA,HDOP,VDOP",
                                "s-----------",
                                "F-----------",
                                "QIHHBBBfffff",
                                AP_HAL::micros64(),
                                gps_msg.ms_tow,
                                gps_msg.gps_week,
                                num_sats,
                                uint8_t(gps_msg.fix_type),
                                jam, spf,
                                shared.cust_hpa_m, shared.cust_vpa_m, shared.cust_hva_mps,
                                shared.gps_hdop, shared.cust_vdop);
#endif
}

void AP_ExternalAHRS_Aeron_plx::send_deferred_messages(const AeronDeferredMsgs &msgs)
{
    if (msgs.crc_fail) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Aeron: CRC mismatch");
    }
    if (msgs.garbage) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Aeron: Unknown packet 0x%04X", msgs.unknown_id);
    }
    if (msgs.ext_mag_checked) {
        uint8_t present;
        {
            WITH_SEMAPHORE(state.sem);
            present = shared.ext_mag_present;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Ext mag present=%u", present);
    }
    if (msgs.ext_asp_checked) {
        uint8_t present;
        {
            WITH_SEMAPHORE(state.sem);
            present = shared.ext_asp_present;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Ext airspeed present=%u", present);
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
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: External sensor query sent");
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
    return last_sens_ms != 0
        && last_nav_ms  != 0
        && last_gnss_ms != 0;
}

/*
  Hardware-status reporting. Emits GCS warnings for any error bit in
  NAV_PARA2.hw_status, with rising-edge + per-bit-rate-limited
  semantics.

  Reporting strategy:
    - Rising edge (bit 0 -> 1):       report immediately, once.
    - Persistent fault (bit stays 1): repeat every HEALTH_REPEAT_INTERVAL_MS.
    - Falling edge (bit 1 -> 0):      log "okay" and stop reporting.

  The hw_status bits do NOT fail healthy() - the PLX continues
  navigation with degraded sensor inputs and reports the resulting
  accuracy degradation through get_variances(). This routine only
  emits informational/warning GCS messages.
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
        WITH_SEMAPHORE(state.sem);
        hw              = shared.nav2_hw_status;
        ext_mag_present = shared.ext_mag_present;
        ext_asp_present = shared.ext_asp_present;
        ext_mag_checked = shared.ext_mag_checked;
        ext_asp_checked = shared.ext_asp_checked;
    }

    // Drop EXT_* error bits unless the user configured the sensor.
    uint32_t effective_mask = HW_ERROR_MASK;
    if (!(ext_mag_checked && ext_mag_present)) {
        effective_mask &= ~(1U << uint8_t(AeronHwBit::EXT_MAG));
    }
    if (!(ext_asp_checked && ext_asp_present)) {
        effective_mask &= ~(1U << uint8_t(AeronHwBit::EXT_ASP));
    }

    const uint32_t errors      = hw & effective_mask;
    const uint32_t newly_set   = errors & ~last_warned_bits;
    const uint32_t newly_clear = last_warned_bits & ~errors;

    // log clears for any bits that just dropped
    for (uint8_t index = 0; (index < HW_BIT_COUNT) && (newly_clear != 0U); index++) {
        if ((newly_clear & (1U << index)) == 0U) {
            continue;
        }
        switch (index) {
        case uint8_t(AeronHwBit::GNSS):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: GNSS okay"); 
            break;
        case uint8_t(AeronHwBit::ACC):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Accel okay"); 
            break;
        case uint8_t(AeronHwBit::GYR):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Gyro okay"); 
            break;
        case uint8_t(AeronHwBit::MAG):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Mag okay"); 
            break;
        case uint8_t(AeronHwBit::BARO):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Baro okay"); 
            break;
        case uint8_t(AeronHwBit::SUP_VLTG):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Supply voltage okay"); 
            break;
        case uint8_t(AeronHwBit::CM_PRT):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Comport okay"); 
            break;
        case uint8_t(AeronHwBit::EXT_MAG):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Ext Mag connected"); 
            break;
        case uint8_t(AeronHwBit::EXT_ASP):
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Aeron: Ext Airspeed connected"); 
            break;
        default:
            // RAM/FIRMWARE/CONFIG don't meaningfully "recover".
            break;
        }
    }

    // walk every currently-set error bit. Report it if either:
    //   - it is a rising edge (newly_set), or
    //   - HEALTH_REPEAT_INTERVAL_MS has elapsed since the last warning.
    for (uint8_t index = 0; index < HW_BIT_COUNT; index++) {
        const uint32_t mask = 1U << index;
        if ((errors & mask) == 0U) {
            continue;
        }
        const bool rising  = (newly_set & mask) != 0U;
        const bool overdue = (now - last_warned_ms[index]) >= HEALTH_REPEAT_INTERVAL_MS;
        if (!rising && !overdue) {
            continue;
        }

        switch (index) {
        case uint8_t(AeronHwBit::GNSS):
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,    "Aeron: GNSS unhealthy"); 
            break;
        case uint8_t(AeronHwBit::ACC):
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,     "Aeron: Accelerometer unhealthy"); 
            break;
        case uint8_t(AeronHwBit::GYR):
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,     "Aeron: Gyroscope unhealthy"); 
            break;
        case uint8_t(AeronHwBit::MAG):
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,     "Aeron: Magnetometer unhealthy"); 
            break;
        case uint8_t(AeronHwBit::BARO):
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,     "Aeron: Barometer unhealthy"); 
            break;
        case uint8_t(AeronHwBit::SUP_VLTG):
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,   "Aeron: Supply Voltage Error"); 
            break;
        case uint8_t(AeronHwBit::CM_PRT):
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,   "Aeron: Comport Overrun"); 
            break;
        case uint8_t(AeronHwBit::RAM):
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Aeron: RAM Failure"); 
            break;
        case uint8_t(AeronHwBit::FIRMWARE):
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Aeron: Firmware Failure"); 
            break;
        case uint8_t(AeronHwBit::CONFIG):
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Aeron: Config Memory Failure"); 
            break;
        case uint8_t(AeronHwBit::EXT_MAG):
            // gated above by effective_mask: only reaches here if user
            // configured ext mag AND it is missing
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Aeron: ext mag not connected"); 
            break;
        case uint8_t(AeronHwBit::EXT_ASP):
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Aeron: ext airspeed not connected"); 
            break;
        default:
            // Reserved bits inside HW_BIT_COUNT but not in the enum.
            // Should not be reachable since HW_ERROR_MASK only sets
            // the enum bits. Skip without updating timestamp.
            continue;
        }

        last_warned_ms[index] = now;
    }

    last_warned_bits = errors;
}

/*
  Health check. Pure stale-timestamp test.
 */
bool AP_ExternalAHRS_Aeron_plx::healthy() const
{
    const uint32_t now = AP_HAL::millis();
    return (now - last_sens_ms) < SENS_TIMEOUT_MS
        && (now - last_nav_ms)  < NAV_TIMEOUT_MS
        && (now - last_gnss_ms) < GNSS_TIMEOUT_MS;
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
  Filter status. horiz_pos_abs / pred_horiz_pos_abs are gated on having
  a current GNSS fix - otherwise the position is INS-only,
  and downstream consumers must not treat it as bounded-error absolute.
  horiz_pos_rel stays true to indicate that relative position estimation
  is still functional via dead reckoning.
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
        WITH_SEMAPHORE(state.sem);
        fix = gnss_fix;
    }

    status.flags.attitude            = true;
    status.flags.horiz_vel           = true;
    status.flags.vert_vel            = true;
    status.flags.horiz_pos_rel       = true;
    status.flags.vert_pos            = true;
    status.flags.pred_horiz_pos_rel  = true;
    status.flags.using_gps           = fix;
    status.flags.gps_quality_good    = fix;

    if (fix) {
        status.flags.horiz_pos_abs      = true;
        status.flags.pred_horiz_pos_abs = true;
    }
}

/*
  Variance estimates from EXTD_GNSS accuracy metrics. Returns false
  until the first EXTD_GNSS packet arrives so ArduPilot does not
  over-trust the solution during initialisation when accuracy fields
  are still zero.
 */
bool AP_ExternalAHRS_Aeron_plx::get_variances(float &velVar, float &posVar,
                                              float &hgtVar, Vector3f &magVar,
                                              float &tasVar) const
{
    if (!has_variance_data) {
        return false;
    }

    float hpa;
    float vpa;
    float hva;
    {
        WITH_SEMAPHORE(state.sem);
        hpa = shared.cust_hpa_m;
        vpa = shared.cust_vpa_m;
        hva = shared.cust_hva_mps;
    }

    velVar = hva * vel_gate_scale;
    posVar = hpa * pos_gate_scale;
    hgtVar = vpa * hgt_gate_scale;
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_AERON_PLX_ENABLED
