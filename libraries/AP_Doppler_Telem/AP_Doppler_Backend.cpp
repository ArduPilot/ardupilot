#include "AP_Doppler_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RPM/AP_RPM.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

namespace {
constexpr uint8_t AP_DOPPLER_LINE_BUFFER_SIZE = 128;

static inline uint32_t parse_u32(const char *&p)
{
    while (*p == ' ') {
        ++p;
    }

    char *end = nullptr;
    const unsigned long v = strtoul(p, &end, 10);
    if (end == p) {
        const char *q = p;
        while (*q && *q != ',') {
            ++q;
        }
        p = q;
        if (*p == ',') {
            ++p;
        }
        return 0;
    }

    p = end;
    if (*p == ',') {
        ++p;
    }
    return uint32_t(v);
}

static inline uint64_t parse_u64(const char *&p)
{
    while (*p == ' ') {
        ++p;
    }

    char *end = nullptr;
    const uint64_t v = strtoull(p, &end, 10);
    if (end == p) {
        const char *q = p;
        while (*q && *q != ',') {
            ++q;
        }
        p = q;
        if (*p == ',') {
            ++p;
        }
        return 0;
    }

    p = end;
    if (*p == ',') {
        ++p;
    }
    return v;
}
}

bool AP_Doppler_Backend::init()
{
    return init_serial_port();
}

bool AP_Doppler_Backend::init_serial_port()
{
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_Doppler_Backend::loop, void),
            "Doppler",
            1024,
            AP_HAL::Scheduler::PRIORITY_UART,
            1)) {
        return false;
    }
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    gcs().send_text(MAV_SEVERITY_INFO, "Doppler Telemetry Initialized");
    return true;
}

void AP_Doppler_Backend::loop(void)
{
    if (_port == nullptr) {
        return;
    }

    _port->begin(AP_DOPPLER_BAUD, AP_DOPPLER_BUFSIZE_RX, AP_DOPPLER_BUFSIZE_TX);
    send_epd6_startup_commands();

    char buffer[AP_DOPPLER_LINE_BUFFER_SIZE];
    uint8_t idx = 0;
    bool in_frame = false;

    while (true) {
        if (_port->available() == 0) {
            _port->write((const uint8_t *)AP_DOPPLER_LAUNCH, AP_DOPPLER_CMD_LENGTH);
            hal.scheduler->delay(100);
            continue;
        }

        while (_port->available() != 0) {
            const char c = _port->read();

            if (!in_frame) {
                if (c == AP_DOPPLER_START_BYTE) {
                    in_frame = true;
                    idx = 0;
                }
                continue;
            }

            if (c == AP_DOPPLER_START_BYTE) {
                idx = 0;
                continue;
            }

            if (c == '\n' || c == '\r') {
                if (idx == 0) {
                    in_frame = false;
                    continue;
                }

                buffer[idx] = '\0';
                if (strncmp(buffer, "SA", 2) == 0) {
                    parse_epd6_sa(buffer);
                } else if (strncmp(buffer, "TS", 2) == 0) {
                    parse_epd6_ts(buffer);
                } else if (strncmp(buffer, "BI", 2) == 0) {
                    parse_epd6_bi(buffer);
                } else if (strncmp(buffer, "BS", 2) == 0) {
                    parse_epd6_bs(buffer);
                } else if (strncmp(buffer, "BE", 2) == 0) {
                    parse_epd6_be(buffer);
                } else if (strncmp(buffer, "BD", 2) == 0) {
                    parse_epd6_bd(buffer);
                } else if (strncmp(buffer, "WI", 2) == 0) {
                    parse_epd6_wi(buffer);
                } else if (strncmp(buffer, "WS", 2) == 0) {
                    parse_epd6_ws(buffer);
                } else if (strncmp(buffer, "WE", 2) == 0) {
                    parse_epd6_we(buffer);
                } else if (strncmp(buffer, "WD", 2) == 0) {
                    parse_epd6_wd(buffer);
                } else if (strncmp(buffer, "UA", 2) == 0) {
                    parse_epd6_ua(buffer);
                } else if (strncmp(buffer, "UB", 2) == 0) {
                    parse_epd6_ub(buffer);
                } else if (strncmp(buffer, "UC", 2) == 0) {
                    parse_epd6_uc(buffer);
                } else if (strncmp(buffer, "UD", 2) == 0) {
                    parse_epd6_ud(buffer);
                } else if (strncmp(buffer, "TD", 2) == 0) {
                    parse_epd6_td(buffer);
                }

                idx = 0;
                in_frame = false;
                continue;
            }

            if (idx < AP_DOPPLER_LINE_BUFFER_SIZE - 1) {
                buffer[idx++] = c;
            } else {
                idx = 0;
                in_frame = false;
            }
        }
    }
}

void AP_Doppler_Backend::send()
{
    if (_port == nullptr) {
        return;
    }
}

bool AP_Doppler_Backend::get_bi_msg(DVL_BI_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = bi_msg;
    return msg.valid;
}

bool AP_Doppler_Backend::get_bd_msg(DVL_BD_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = bd_msg;
    return msg.valid;
}

bool AP_Doppler_Backend::get_wi_msg(DVL_WI_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = wi_msg;
    return msg.valid;
}

bool AP_Doppler_Backend::get_ua_msg(DVL_U_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = ua_msg;
    return msg.valid;
}

bool AP_Doppler_Backend::get_ub_msg(DVL_U_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = ub_msg;
    return msg.valid;
}

bool AP_Doppler_Backend::get_uc_msg(DVL_U_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = uc_msg;
    return msg.valid;
}

bool AP_Doppler_Backend::get_ud_msg(DVL_U_Msg &msg) const
{
    WITH_SEMAPHORE(_sample_sem);
    msg = ud_msg;
    return msg.valid;
}

void AP_Doppler_Backend::send_epd6_startup_commands()
{
    if (_port == nullptr) {
        return;
    }

    _port->write((const uint8_t *)AP_DOPPLER_SET_EPD6, AP_DOPPLER_CMD_LENGTH);
    hal.scheduler->delay(50);
    _port->write((const uint8_t *)AP_DOPPLER_LAUNCH, AP_DOPPLER_CMD_LENGTH);
}

static inline float parse_float(const char *&p)
{
    while (*p == ' ') {
        ++p;
    }

    char *end = nullptr;
    float v = strtof(p, &end);
    if (end == p) {
        v = NAN;
        const char *q = p;
        while (*q && *q != ',') {
            ++q;
        }
        p = q;
        if (*p == ',') {
            ++p;
        }
        return v;
    }

    p = end;
    if (*p == ',') {
        ++p;
    }
    return v;
}

static inline char parse_char(const char *&p)
{
    while (*p == ' ') {
        ++p;
    }

    const char c = *p;
    if (c == '\0' || c == '\r' || c == '\n') {
        return '\0';
    }

    ++p;
    if (*p == ',') {
        ++p;
    }

    return c;
}

void AP_Doppler_Backend::parse_epd6_sa(const char *payload)
{
    const char *p = payload + 3;
    _epd6_sa.roll_deg  = parse_float(p);
    _epd6_sa.pitch_deg = parse_float(p);
    _epd6_sa.yaw_deg   = parse_float(p);
}

void AP_Doppler_Backend::parse_epd6_ts(const char *payload)
{
    const char *p = payload + 3;

    char *comma = strchr(p, ',');
    if (comma == nullptr) {
        return;
    }

    size_t len = comma - p;
    char time_buf[sizeof(_epd6_ts.time)];
    if (len >= sizeof(time_buf)) {
        len = sizeof(time_buf) - 1;
    }
    memcpy(time_buf, p, len);
    time_buf[len] = '\0';
    p = comma + 1;

    const float satellites_PPT = parse_float(p);
    const float temperature_C = parse_float(p);
    const float deep_m = parse_float(p);
    const float sound_speed_m_s = parse_float(p);

    while (*p == ' ') {
        ++p;
    }
    char status_buf[4] {};
    size_t status_len = 0;
    while (*p != '\0' && *p != '\r' && *p != '\n' && status_len < 3) {
        if (*p != ',' && *p != ' ') {
            status_buf[status_len++] = *p;
        }
        ++p;
    }

    memcpy(_epd6_ts.time, time_buf, sizeof(time_buf));
    _epd6_ts.satellites_PPT = satellites_PPT;
    _epd6_ts.temperature_C = temperature_C;
    _epd6_ts.deep_m = deep_m;
    _epd6_ts.sound_speed_m_s = sound_speed_m_s;
    _epd6_ts.fault_code = (status_len > 0 && status_buf[0] >= '0' && status_buf[0] <= '9') ? uint8_t(status_buf[0] - '0') : 0;
    _epd6_ts.protocol_version[0] = (status_len > 1) ? status_buf[1] : '\0';
    _epd6_ts.protocol_version[1] = (status_len > 2) ? status_buf[2] : '\0';
    _epd6_ts.protocol_version[2] = '\0';
}

void AP_Doppler_Backend::parse_epd6_bi(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    const float velocity_error_mm_s = parse_float(p);
    _epd6_bi.status = static_cast<EPD6_Status>(parse_char(p));
    if (_epd6_bi.status == EPD6_STATUS_ACQUIRING) {
        _epd6_bi.X_velocity_mm_s = x_velocity_mm_s;
        _epd6_bi.Y_velocity_mm_s = y_velocity_mm_s;
        _epd6_bi.Z_velocity_mm_s = z_velocity_mm_s;
        _epd6_bi.velocity_error_mm_s = velocity_error_mm_s;
    }

    WITH_SEMAPHORE(_sample_sem);
    bi_msg.time_usec = AP_HAL::micros64();
    bi_msg.sequence++;
    bi_msg.valid = (_epd6_bi.status == EPD6_STATUS_ACQUIRING);
    bi_msg.status = uint8_t(_epd6_bi.status);
    bi_msg.vx_mps = x_velocity_mm_s * 0.001f;
    bi_msg.vy_mps = y_velocity_mm_s * 0.001f;
    bi_msg.vz_mps = z_velocity_mm_s * 0.001f;
    bi_msg.vel_error_mps = velocity_error_mm_s * 0.001f;

}

void AP_Doppler_Backend::parse_epd6_bs(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    _epd6_bs.status = static_cast<EPD6_Status>(parse_char(p));
    if (_epd6_bs.status == EPD6_STATUS_ACQUIRING) {
        _epd6_bs.x_velocity_mm_s = x_velocity_mm_s;
        _epd6_bs.y_velocity_mm_s = y_velocity_mm_s;
        _epd6_bs.z_velocity_mm_s = z_velocity_mm_s;
        update_epd6_velocity_sample(_epd6_bottom_track_velocity_sample, x_velocity_mm_s, y_velocity_mm_s, z_velocity_mm_s, DVL_LockState::BOTTOM_LOCK);
    }
}

void AP_Doppler_Backend::parse_epd6_be(const char *payload)
{
    const char *p = payload + 3;
    const float east_velocity_mm_s = parse_float(p);
    const float north_velocity_mm_s = parse_float(p);
    const float up_velocity_mm_s = parse_float(p);
    _epd6_be.status = static_cast<EPD6_Status>(parse_char(p));
    if (_epd6_be.status == EPD6_STATUS_ACQUIRING) {
        _epd6_be.east_velocity_mm_s = east_velocity_mm_s;
        _epd6_be.north_velocity_mm_s = north_velocity_mm_s;
        _epd6_be.up_velocity_mm_s = up_velocity_mm_s;
    }
}

void AP_Doppler_Backend::parse_epd6_bd(const char *payload)
{
    const char *p = payload + 3;
    _epd6_bd.east_distance_m = parse_float(p);
    _epd6_bd.north_distance_m = parse_float(p);
    _epd6_bd.up_distance_m = parse_float(p);
    _epd6_bd.bottom_distance_m = parse_float(p);
    _epd6_bd.time_since_valid_s = parse_float(p);

    WITH_SEMAPHORE(_sample_sem);
    bd_msg.time_usec = AP_HAL::micros64();
    bd_msg.sequence++;
    bd_msg.valid = true;
    bd_msg.status = uint8_t(_epd6_bs.status);
    bd_msg.east_m = _epd6_bd.east_distance_m;
    bd_msg.north_m = _epd6_bd.north_distance_m;
    bd_msg.up_m = _epd6_bd.up_distance_m;
    bd_msg.bottom_m = _epd6_bd.bottom_distance_m;
    bd_msg.time_since_valid_s = _epd6_bd.time_since_valid_s;
}

void AP_Doppler_Backend::parse_epd6_wi(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    const float velocity_error_mm_s = parse_float(p);
    _epd6_wi.status = static_cast<EPD6_Status>(parse_char(p));
    if (_epd6_wi.status == EPD6_STATUS_ACQUIRING) {
        _epd6_wi.x_velocity_mm_s = x_velocity_mm_s;
        _epd6_wi.y_velocity_mm_s = y_velocity_mm_s;
        _epd6_wi.z_velocity_mm_s = z_velocity_mm_s;
        _epd6_wi.velocity_error_mm_s = velocity_error_mm_s;
    }

    WITH_SEMAPHORE(_sample_sem);
    wi_msg.time_usec = AP_HAL::micros64();
    wi_msg.sequence++;
    wi_msg.valid = (_epd6_wi.status == EPD6_STATUS_ACQUIRING);
    wi_msg.status = uint8_t(_epd6_wi.status);
    wi_msg.vx_mps = x_velocity_mm_s * 0.001f;
    wi_msg.vy_mps = y_velocity_mm_s * 0.001f;
    wi_msg.vz_mps = z_velocity_mm_s * 0.001f;
    wi_msg.vel_error_mps = velocity_error_mm_s * 0.001f;
}

void AP_Doppler_Backend::parse_epd6_ws(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    _epd6_ws.status = static_cast<EPD6_Status>(parse_char(p));
    if (_epd6_ws.status == EPD6_STATUS_ACQUIRING) {
        _epd6_ws.x_velocity_mm_s = x_velocity_mm_s;
        _epd6_ws.y_velocity_mm_s = y_velocity_mm_s;
        _epd6_ws.z_velocity_mm_s = z_velocity_mm_s;
        update_epd6_velocity_sample(_epd6_water_track_velocity_sample, x_velocity_mm_s, y_velocity_mm_s, z_velocity_mm_s, DVL_LockState::WATER_TRACK);
    }
}

void AP_Doppler_Backend::parse_epd6_we(const char *payload)
{
    const char *p = payload + 3;
    const float east_velocity_mm_s = parse_float(p);
    const float north_velocity_mm_s = parse_float(p);
    const float up_velocity_mm_s = parse_float(p);
    _epd6_we.status = static_cast<EPD6_Status>(parse_char(p));
    if (_epd6_we.status == EPD6_STATUS_ACQUIRING) {
        _epd6_we.east_velocity_mm_s = east_velocity_mm_s;
        _epd6_we.north_velocity_mm_s = north_velocity_mm_s;
        _epd6_we.up_velocity_mm_s = up_velocity_mm_s;
    }
}

void AP_Doppler_Backend::parse_epd6_wd(const char *payload)
{
    const char *p = payload + 3;
    _epd6_wd.east_distance_m = parse_float(p);
    _epd6_wd.north_distance_m = parse_float(p);
    _epd6_wd.up_distance_m = parse_float(p);
    _epd6_wd.center_distance_m = parse_float(p);
    _epd6_wd.time_since_valid_s = parse_float(p);
}

void AP_Doppler_Backend::parse_epd6_ua(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const EPD6_Status status = static_cast<EPD6_Status>(parse_char(p));
    update_epd6_beam_sample(_epd6_beam_samples[0], velocity_mm_s, distance_m, rssi, nsd, status);

    WITH_SEMAPHORE(_sample_sem);
    ua_msg.time_usec = AP_HAL::micros64();
    ua_msg.sequence++;
    ua_msg.valid = (status == EPD6_STATUS_ACQUIRING);
    ua_msg.status = uint8_t(status);
    ua_msg.velocity_mps = velocity_mm_s * 0.001f;
    ua_msg.distance_m = distance_m;
    ua_msg.rssi = rssi;
    ua_msg.nsd = nsd;
}

void AP_Doppler_Backend::parse_epd6_ub(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const EPD6_Status status = static_cast<EPD6_Status>(parse_char(p));
    update_epd6_beam_sample(_epd6_beam_samples[1], velocity_mm_s, distance_m, rssi, nsd, status);

    WITH_SEMAPHORE(_sample_sem);
    ub_msg.time_usec = AP_HAL::micros64();
    ub_msg.sequence++;
    ub_msg.valid = (status == EPD6_STATUS_ACQUIRING);
    ub_msg.status = uint8_t(status);
    ub_msg.velocity_mps = velocity_mm_s * 0.001f;
    ub_msg.distance_m = distance_m;
    ub_msg.rssi = rssi;
    ub_msg.nsd = nsd;
}

void AP_Doppler_Backend::parse_epd6_uc(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const EPD6_Status status = static_cast<EPD6_Status>(parse_char(p));
    update_epd6_beam_sample(_epd6_beam_samples[2], velocity_mm_s, distance_m, rssi, nsd, status);

    WITH_SEMAPHORE(_sample_sem);
    uc_msg.time_usec = AP_HAL::micros64();
    uc_msg.sequence++;
    uc_msg.valid = (status == EPD6_STATUS_ACQUIRING);
    uc_msg.status = uint8_t(status);
    uc_msg.velocity_mps = velocity_mm_s * 0.001f;
    uc_msg.distance_m = distance_m;
    uc_msg.rssi = rssi;
    uc_msg.nsd = nsd;
}

void AP_Doppler_Backend::parse_epd6_ud(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const EPD6_Status status = static_cast<EPD6_Status>(parse_char(p));
    update_epd6_beam_sample(_epd6_beam_samples[3], velocity_mm_s, distance_m, rssi, nsd, status);

    WITH_SEMAPHORE(_sample_sem);
    ud_msg.time_usec = AP_HAL::micros64();
    ud_msg.sequence++;
    ud_msg.valid = (status == EPD6_STATUS_ACQUIRING);
    ud_msg.status = uint8_t(status);
    ud_msg.velocity_mps = velocity_mm_s * 0.001f;
    ud_msg.distance_m = distance_m;
    ud_msg.rssi = rssi;
    ud_msg.nsd = nsd;
}

void AP_Doppler_Backend::parse_epd6_td(const char *payload)
{
    const char *p = payload + 3;
    const uint32_t period_ms = parse_u32(p);
    const uint64_t echo_unix_ms = parse_u64(p);
    const uint64_t report_unix_ms = parse_u64(p);

    WITH_SEMAPHORE(_sample_sem);
    _epd6_td.period_ms = period_ms;
    _epd6_td.echo_unix_ms = echo_unix_ms;
    _epd6_td.report_unix_ms = report_unix_ms;
    _epd6_td.valid = true;
    _epd6_td.update_ms = AP_HAL::millis();
}

void AP_Doppler_Backend::update_epd6_velocity_sample(EPD6VelocitySample &sample, float x_velocity_mm_s, float y_velocity_mm_s, float z_velocity_mm_s, DVL_LockState lock)
{
    WITH_SEMAPHORE(_sample_sem);
    sample.vel_body_mps.x = x_velocity_mm_s * 0.001f;
    sample.vel_body_mps.y = y_velocity_mm_s * 0.001f;
    sample.vel_body_mps.z = z_velocity_mm_s * 0.001f;
    sample.update_ms = AP_HAL::millis();
    sample.quality = 255.0f;
    sample.lock = lock;
    sample.valid = true;
}

void AP_Doppler_Backend::update_epd6_beam_sample(EPD6BeamSample &sample, float velocity_mm_s, float distance_m, float rssi, float nsd, EPD6_Status status)
{
    WITH_SEMAPHORE(_sample_sem);
    sample.status = status;
    sample.update_ms = AP_HAL::millis();
    if (status == EPD6_STATUS_ACQUIRING) {
        sample.velocity_mm_s = velocity_mm_s;
        sample.distance_m = distance_m;
        sample.rssi = rssi;
        sample.nsd = nsd;
    }
}

bool AP_Doppler_Backend::get_velocity_body(Vector3f &vel_body_mps, uint32_t &t_ms, float &quality, DVL_LockState &lock) const
{
    lock = DVL_LockState::NO_LOCK;
    quality = 0.0f;
    t_ms = 0;

    const uint32_t now_ms = AP_HAL::millis();
    WITH_SEMAPHORE(_sample_sem);

    if (_epd6_bottom_track_velocity_sample.valid &&
        (now_ms - _epd6_bottom_track_velocity_sample.update_ms) <= DVL_TIMEOUT_MS) {
        vel_body_mps = _epd6_bottom_track_velocity_sample.vel_body_mps;
        t_ms = _epd6_bottom_track_velocity_sample.update_ms;
        quality = _epd6_bottom_track_velocity_sample.quality;
        lock = _epd6_bottom_track_velocity_sample.lock;
        return true;
    }

    if (_epd6_water_track_velocity_sample.valid &&
        (now_ms - _epd6_water_track_velocity_sample.update_ms) <= DVL_TIMEOUT_MS) {
        vel_body_mps = _epd6_water_track_velocity_sample.vel_body_mps;
        t_ms = _epd6_water_track_velocity_sample.update_ms;
        quality = _epd6_water_track_velocity_sample.quality;
        lock = _epd6_water_track_velocity_sample.lock;
        return true;
    }

    return false;
}
