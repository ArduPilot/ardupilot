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
  SITL simulation of Aeron Systems PLX3 series INS
*/

#include "SIM_Aeron.h"

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

using namespace SITL;

Aeron::Aeron() :
    SerialDevice::SerialDevice(32768, 512)
{
}

/*
  produce a timeval that advances with the simulation clock
 */
void Aeron::simulation_timeval(struct timeval *tv)
{
    const uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    const uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

/*
  pack the GPS status word consumed by AP_ExternalAHRS_Aeron::format_and_push_gps()

      bits  0..9  : satellites in use         (mask 0x000003FF)
      bit  10     : fix present               (mask 0x00000400)
      bits 11..14 : raw fix type              (mask 0x00007800)
      bits 16..17 : jamming detection level   (mask 0x00030000)
      bits 18..19 : spoofing detection level  (mask 0x000C0000)
 */
uint32_t Aeron::build_gps_status(uint16_t num_sats, bool fix_present,
                                 uint8_t fix_raw, uint8_t jam, uint8_t spoof)
{
    uint32_t status = uint32_t(num_sats) & 0x3FFu;
    if (fix_present) {
        status |= (1u << 10);
    }
    status |= (uint32_t(fix_raw)  & 0x0Fu) << 11;
    status |= (uint32_t(jam)      & 0x03u) << 16;
    status |= (uint32_t(spoof)    & 0x03u) << 18;
    return status;
}

// Build a complete Aeron frame in a stack buffer and emit it as one
// contiguous write_to_autopilot() call.
void Aeron::send_aeron_packet(PktID prp_id, const void *payload,
                              uint16_t payload_len)
{
    // 4 sync + 2 length + 2 PRP_ID + payload + 1 CRC
    uint8_t buf[8 + sizeof(NavPara2Payload) + 1];
    const uint16_t total_len = 8 + payload_len;          // length field (excludes CRC)
    const uint16_t on_wire   = total_len + 1;            // bytes actually transmitted

    // sanity guard against a future enum getting out of step with the
    // structs.  Panic rather than silently dropping
    if (on_wire > sizeof(buf)) {
        AP_HAL::panic("Aeron SIM: packet too large for tx buffer (%u > %u)",
                      unsigned(on_wire), unsigned(sizeof(buf)));
    }

    // sync bytes
    for (uint8_t index = 0; index < SYNC_COUNT; index++) {
        buf[index] = SYNC_BYTE;
    }

    // length (BE)
    buf[4] = uint8_t(total_len >> 8);
    buf[5] = uint8_t(total_len & 0xFF);

    // PRP_ID (BE)
    const uint16_t id = uint16_t(prp_id);
    buf[6] = uint8_t(id >> 8);
    buf[7] = uint8_t(id & 0xFF);

    // payload (LE on both SITL and the MCU)
    if (payload_len > 0) {
        memcpy(&buf[8], payload, payload_len);
    }

    // CRC = XOR over [0 .. total_len-1]
    buf[total_len] = crc_xor_of_bytes(buf, total_len);

    write_to_autopilot((const char *)buf, on_wire);
}

// NAV_PARA1: fused euler/course/velocity/position.
void Aeron::send_nav_para1()
{
    const auto &fdm = _sitl->state;

    NavPara1Payload p {};

    struct timeval tv;
    simulation_timeval(&tv);
    p.epoch_time   = uint32_t(tv.tv_sec);
    p.microseconds = uint32_t(tv.tv_usec);
    p.ins_status   = 0;
    p.hw_status    = 0;

    // euler [rad] — convert from FDM degrees
    p.euler[0] = radians(fdm.rollDeg);
    p.euler[1] = radians(fdm.pitchDeg);
    p.euler[2] = radians(fdm.yawDeg);

    // course over ground [deg], wrapped 0..360
    const float track_deg = degrees(atan2f(fdm.speedE, fdm.speedN));
    p.course = wrap_360(track_deg);

    p.velocity_ned[0] = fdm.speedN;
    p.velocity_ned[1] = fdm.speedE;
    p.velocity_ned[2] = fdm.speedD;

    p.position[0] = fdm.latitude;
    p.position[1] = fdm.longitude;

    // FDM altitude is MSL; PLX3 reports height-above-ellipsoid. With no
    // simulated geoid model we treat undulation = 0 (GPS_PARA matches).
    p.height_abv_ellip = fdm.altitude;

    send_aeron_packet(PktID::NAV_PARA1, &p, sizeof(p));
}

// NAV_PARA2: quaternion, body-frame velocity, ECEF position.
void Aeron::send_nav_para2()
{
    const auto &fdm = _sitl->state;

    NavPara2Payload p {};

    struct timeval tv;
    simulation_timeval(&tv);
    p.epoch_time   = uint32_t(tv.tv_sec);
    p.microseconds = uint32_t(tv.tv_usec);
    p.ins_status   = 0;
    p.hw_status    = 0;

    // Quaternion in (w, x, y, z) order, matching the driver's
    p.quat[0] = fdm.quaternion.q1;
    p.quat[1] = fdm.quaternion.q2;
    p.quat[2] = fdm.quaternion.q3;
    p.quat[3] = fdm.quaternion.q4;

    // rotate NED velocity into body frame
    Vector3f v_body { float(fdm.speedN), float(fdm.speedE), float(fdm.speedD) };
    fdm.quaternion.earth_to_body(v_body);
    p.body_vel[0] = v_body.x;
    p.body_vel[1] = v_body.y;
    p.body_vel[2] = v_body.z;

    // compute ECEF using the AP_Math helper packed in a Vector3d.
    Vector3d llh{radians(fdm.latitude), radians(fdm.longitude), double(fdm.altitude)};
    Vector3d ecef_v;
    wgsllh2ecef(llh, ecef_v);
    p.ecef_pos[0] = ecef_v.x;
    p.ecef_pos[1] = ecef_v.y;
    p.ecef_pos[2] = ecef_v.z;

    p.altitude = fdm.altitude;

    send_aeron_packet(PktID::NAV_PARA2, &p, sizeof(p));
}

// SENS_PARA: raw IMU / mag / baro. Rate-critical for EAHRS_RATE.
void Aeron::send_sens_para()
{
    const auto &fdm = _sitl->state;

    SensParaPayload p {};

    struct timeval tv;
    simulation_timeval(&tv);
    p.epoch_time   = uint32_t(tv.tv_sec);
    p.microseconds = uint32_t(tv.tv_usec);

    // synthesise pressure & temperature from altitude
    float pressure_pa = 0.0f;
    float temp_K      = 0.0f;
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude, pressure_pa, temp_K);

    p.temperature = KELVIN_TO_C(temp_K);

    // gyro [deg/s] — FDM rates are already in deg/s, body frame
    p.gyro[0] = fdm.rollRate;
    p.gyro[1] = fdm.pitchRate;
    p.gyro[2] = fdm.yawRate;

    // accel [m/s²] body frame
    p.accel[0] = fdm.xAccel;
    p.accel[1] = fdm.yAccel;
    p.accel[2] = fdm.zAccel;

    // magnetic field [mGauss] body frame — bodyMagField is already in mGauss
    p.mag[0] = fdm.bodyMagField.x;
    p.mag[1] = fdm.bodyMagField.y;
    p.mag[2] = fdm.bodyMagField.z;

    // Small-angle approximation - the driver doesn't use these fields,
    // so the singularity near pitch = +/- 90 degrees is irrelevant.
    p.euler_rates[0] = fdm.rollRate;
    p.euler_rates[1] = fdm.pitchRate;
    p.euler_rates[2] = fdm.yawRate;

    p.baro_pressure    = pressure_pa + rand_float() * 0.01f;
    p.baro_temperature = KELVIN_TO_C(temp_K) + rand_float() * 0.001f;
    p.baro_altitude    = fdm.altitude;

    send_aeron_packet(PktID::SENS_PARA, &p, sizeof(p));
}

/*
  GPS_PARA — GNSS solution, status, DOP
 */
void Aeron::send_gps_para()
{
    const auto &fdm = _sitl->state;

    GpsParaPayload p {};

    struct timeval tv;
    simulation_timeval(&tv);
    p.epoch_time   = uint32_t(tv.tv_sec);
    p.microseconds = uint32_t(tv.tv_usec);

    p.gps_status = build_gps_status(SIM_NUM_SATS,
                                    /*fix_present=*/true,
                                    SIM_FIX_RAW,
                                    SIM_JAM_STATUS,
                                    SIM_SPOOF_LEVEL);

    p.gps_position[0]      = fdm.latitude;
    p.gps_position[1]      = fdm.longitude;
    p.gps_height_abv_ellip = fdm.altitude;
    p.gps_undulation       = 0.0f;
    p.pdop                 = SIM_PDOP;
    p.hdop                 = SIM_HDOP;
    p.gdop                 = SIM_GDOP;

    send_aeron_packet(PktID::GPS_PARA, &p, sizeof(p));
}

// EXTD_GNSS: accuracy metrics + GNSS NED velocity. Drives get_variances()
// and the velocity in the constructed GPS message.
void Aeron::send_extd_gnss()
{
    const auto &fdm = _sitl->state;

    ExtdGnssPayload p {};

    p.hpa  = SIM_HPA_M;
    p.vpa  = SIM_VPA_M;
    p.hva  = SIM_HVA_MPS;
    p.vdop = SIM_VDOP;

    p.gnss_vned[0] = fdm.speedN;
    p.gnss_vned[1] = fdm.speedE;
    p.gnss_vned[2] = fdm.speedD;

    send_aeron_packet(PktID::EXTD_GNSS, &p, sizeof(p));
}

/*
  Send Aeron data — drained at every SIMState step.
 */
void Aeron::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    // 1 kHz tick gate
    const uint64_t tick_now = (AP_HAL::micros64() + 500) / 1000;
    if (tick_now <= _tick) {
        return;
    }
    _tick = tick_now;

    // Drive the inbound parser.  The driver sends EXT_SENSORS_* query
    // commands a few seconds after boot; handle_inbound_packet replies
    // with the matching presence frame so the driver's response path
    // gets exercised in SITL.
    process_inbound();

    // Phased emission.
    if ((_tick % SENS_PERIOD_MS) == SENS_PHASE_MS) {
        send_sens_para();
    } else if ((_tick % NAV1_PERIOD_MS) == NAV1_PHASE_MS) {
        send_nav_para1();
    } else if ((_tick % NAV2_PERIOD_MS) == NAV2_PHASE_MS) {
        send_nav_para2();
    } else if ((_tick % GPS_PERIOD_MS) == GPS_PHASE_MS) {
        send_gps_para();
    } else if ((_tick % EXTD_PERIOD_MS) == EXTD_PHASE_MS) {
        send_extd_gnss();
    }
}

// Drain whatever the driver has sent us this tick.  Each byte feeds a
// minimal state-machine that mirrors the on-wire framing handled by
// AP_ExternalAHRS_Aeron_plx::parse_byte.
void Aeron::process_inbound()
{
    char buf[64];
    ssize_t num;
    while ((num = read_from_autopilot(buf, sizeof(buf))) > 0) {
        for (ssize_t index = 0; index < num; index++) {
            handle_inbound_byte(uint8_t(buf[index]));
        }
    }
}

void Aeron::handle_inbound_byte(uint8_t byte)
{
    switch (rx_state) {
    case RxParseState::SYNC:
        if (byte == SYNC_BYTE) {
            rx_buf[rx_write_idx++] = byte;
            if (++rx_sync_count == SYNC_COUNT) {
                rx_state      = RxParseState::LEN_HIGH;
                rx_sync_count = 0;
            }
        } else {
            rx_sync_count = 0;
            rx_write_idx  = 0;
        }
        return;

    case RxParseState::LEN_HIGH:
        rx_pkt_len = byte;
        rx_buf[rx_write_idx++] = byte;
        rx_state = RxParseState::LEN_LOW;
        return;

    case RxParseState::LEN_LOW:
        rx_buf[rx_write_idx++] = byte;
        rx_pkt_len = (rx_pkt_len << 8) | byte;
        if (rx_pkt_len >= 8 && rx_pkt_len < sizeof(rx_buf)) {
            rx_state = RxParseState::PAYLOAD;
        } else {
            rx_state = RxParseState::RESET;
        }
        return;

    case RxParseState::PAYLOAD:
        rx_buf[rx_write_idx++] = byte;
        if (rx_write_idx >= rx_pkt_len) {
            rx_state = RxParseState::CRC;
        }
        return;

    case RxParseState::CRC: {
        rx_buf[rx_write_idx++] = byte;
        const bool valid = (byte == crc_xor_of_bytes(rx_buf, rx_pkt_len));
        if (valid) {
            const uint16_t prp = (uint16_t(rx_buf[6]) << 8) | rx_buf[7];
            handle_inbound_packet(prp, &rx_buf[8], rx_pkt_len - 8);
        }
        rx_state     = RxParseState::SYNC;
        rx_write_idx = 0;
        rx_pkt_len   = 0;
        return;
    }

    case RxParseState::RESET: {
        rx_write_idx = 0;
        rx_pkt_len = 0;
        rx_state = RxParseState::SYNC;
        return ;
    }
    }

    rx_state = RxParseState::RESET;
    return ;
}

// Dispatch a CRC-valid inbound command.  The PLX query format puts the
// target packet ID in the 2-byte payload (big-endian) of a 0x00A1
// command frame; we reply with the matching response packet carrying a
// single byte of presence info.
void Aeron::handle_inbound_packet(uint16_t prp_id, const uint8_t *payload, uint16_t payload_len)
{
    if (prp_id != QUERY_PRP_ID || payload_len < 2) {
        return;
    }

    const uint16_t target = (uint16_t(payload[0]) << 8) | payload[1];
    const uint8_t  presence = 1;   // SIM always reports the sensor as present

    if (target == uint16_t(PktID::EXT_SENSORS_MAG)) {
        send_aeron_packet(PktID::EXT_SENSORS_MAG, &presence, sizeof(presence));
    } else if (target == uint16_t(PktID::EXT_SENSORS_ASP)) {
        send_aeron_packet(PktID::EXT_SENSORS_ASP, &presence, sizeof(presence));
    }
}
