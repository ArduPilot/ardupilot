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
    _port->write((const uint8_t *)AP_DOPPLER_LAUNCH, 16);

    char buffer[AP_DOPPLER_LINE_BUFFER_SIZE];
    uint8_t idx = 0;
    bool in_frame = false;

    while (true) {
        if (_port->available() == 0) {
            _port->write((const uint8_t *)AP_DOPPLER_LAUNCH, 16);
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
                    parse_SA(buffer);
                } else if (strncmp(buffer, "TS", 2) == 0) {
                    parse_TS(buffer);
                } else if (strncmp(buffer, "BI", 2) == 0) {
                    parse_BI(buffer);
                } else if (strncmp(buffer, "BS", 2) == 0) {
                    parse_BS(buffer);
                } else if (strncmp(buffer, "BE", 2) == 0) {
                    parse_BE(buffer);
                } else if (strncmp(buffer, "BD", 2) == 0) {
                    parse_BD(buffer);
                } else if (strncmp(buffer, "WI", 2) == 0) {
                    parse_WI(buffer);
                } else if (strncmp(buffer, "WS", 2) == 0) {
                    parse_WS(buffer);
                } else if (strncmp(buffer, "WE", 2) == 0) {
                    parse_WE(buffer);
                } else if (strncmp(buffer, "WD", 2) == 0) {
                    parse_WD(buffer);
                } else if (strncmp(buffer, "UA", 2) == 0) {
                    parse_UA(buffer);
                } else if (strncmp(buffer, "UB", 2) == 0) {
                    parse_UB(buffer);
                } else if (strncmp(buffer, "UC", 2) == 0) {
                    parse_UC(buffer);
                } else if (strncmp(buffer, "UD", 2) == 0) {
                    parse_UD(buffer);
                } else if (strncmp(buffer, "TD", 2) == 0) {
                    parse_TD(buffer);
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

void AP_Doppler_Backend::parse_SA(const char *payload)
{
    const char *p = payload + 3;
    Posture_data.pitch_deg = parse_float(p);
    Posture_data.roll_deg  = parse_float(p);
    Posture_data.yaw_deg   = parse_float(p);
}

void AP_Doppler_Backend::parse_TS(const char *payload)
{
    const char *p = payload + 3;

    char *comma = strchr(p, ',');
    if (comma == nullptr) {
        return;
    }

    size_t len = comma - p;
    char time_buf[sizeof(Parameters_data.time)];
    if (len >= sizeof(time_buf)) {
        len = sizeof(time_buf) - 1;
    }
    memcpy(time_buf, p, len);
    time_buf[len] = '\0';
    p = comma + 1;

    const float satellites_PPT = parse_float(p);
    const float temperature_C = parse_float(p);
    const float deep_m = parse_float(p);
    const float voltage_m_s = parse_float(p);
    const char status_char = parse_char(p);
    const uint8_t version = (uint8_t)parse_float(p);

    Parameters_data.status = static_cast<Message_Status>(status_char);
    if (Parameters_data.status == STATUS_ACQUIRING) {
        memcpy(Parameters_data.time, time_buf, sizeof(time_buf));
        Parameters_data.satellites_PPT = satellites_PPT;
        Parameters_data.temperature_C = temperature_C;
        Parameters_data.deep_m = deep_m;
        Parameters_data.voltage_m_s = voltage_m_s;
        Parameters_data.version = version;
    }
}

void AP_Doppler_Backend::parse_BI(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_m_s = parse_float(p);
    const float y_velocity_m_s = parse_float(p);
    const float z_velocity_m_s = parse_float(p);
    const float velocity_error_mm_s = parse_float(p);
    velocity_data.status = static_cast<Message_Status>(parse_char(p));
    if (velocity_data.status == STATUS_ACQUIRING) {
        velocity_data.X_velocity_m_s = x_velocity_m_s;
        velocity_data.Y_velocity_m_s = y_velocity_m_s;
        velocity_data.Z_velocity_m_s = z_velocity_m_s;
        velocity_data.velocity_error_mm_s = velocity_error_mm_s;
    }
}

void AP_Doppler_Backend::parse_BS(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    BottomTrackShipVel.status = static_cast<Message_Status>(parse_char(p));
    if (BottomTrackShipVel.status == STATUS_ACQUIRING) {
        BottomTrackShipVel.x_velocity_mm_s = x_velocity_mm_s;
        BottomTrackShipVel.y_velocity_mm_s = y_velocity_mm_s;
        BottomTrackShipVel.z_velocity_mm_s = z_velocity_mm_s;
        update_velocity_sample(_bottom_track_sample, x_velocity_mm_s, y_velocity_mm_s, z_velocity_mm_s, DVL_LockState::BOTTOM_LOCK);
    }
}

void AP_Doppler_Backend::parse_BE(const char *payload)
{
    const char *p = payload + 3;
    const float east_velocity_mm_s = parse_float(p);
    const float north_velocity_mm_s = parse_float(p);
    const float up_velocity_mm_s = parse_float(p);
    BottomTrackEarthVel.status = static_cast<Message_Status>(parse_char(p));
    if (BottomTrackEarthVel.status == STATUS_ACQUIRING) {
        BottomTrackEarthVel.east_velocity_mm_s = east_velocity_mm_s;
        BottomTrackEarthVel.north_velocity_mm_s = north_velocity_mm_s;
        BottomTrackEarthVel.up_velocity_mm_s = up_velocity_mm_s;
    }
}

void AP_Doppler_Backend::parse_BD(const char *payload)
{
    const char *p = payload + 3;
    BottomTrackDistance.east_distance_m = parse_float(p);
    BottomTrackDistance.north_distance_m = parse_float(p);
    BottomTrackDistance.up_distance_m = parse_float(p);
    BottomTrackDistance.bottom_distance_m = parse_float(p);
    BottomTrackDistance.time_since_valid_s = parse_float(p);
}

void AP_Doppler_Backend::parse_WI(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    const float velocity_error_mm_s = parse_float(p);
    WaterTrackInstrumentVel.status = static_cast<Message_Status>(parse_char(p));
    if (WaterTrackInstrumentVel.status == STATUS_ACQUIRING) {
        WaterTrackInstrumentVel.x_velocity_mm_s = x_velocity_mm_s;
        WaterTrackInstrumentVel.y_velocity_mm_s = y_velocity_mm_s;
        WaterTrackInstrumentVel.z_velocity_mm_s = z_velocity_mm_s;
        WaterTrackInstrumentVel.velocity_error_mm_s = velocity_error_mm_s;
    }
}

void AP_Doppler_Backend::parse_WS(const char *payload)
{
    const char *p = payload + 3;
    const float x_velocity_mm_s = parse_float(p);
    const float y_velocity_mm_s = parse_float(p);
    const float z_velocity_mm_s = parse_float(p);
    WaterTrackShipVel.status = static_cast<Message_Status>(parse_char(p));
    if (WaterTrackShipVel.status == STATUS_ACQUIRING) {
        WaterTrackShipVel.x_velocity_mm_s = x_velocity_mm_s;
        WaterTrackShipVel.y_velocity_mm_s = y_velocity_mm_s;
        WaterTrackShipVel.z_velocity_mm_s = z_velocity_mm_s;
        update_velocity_sample(_water_track_sample, x_velocity_mm_s, y_velocity_mm_s, z_velocity_mm_s, DVL_LockState::WATER_TRACK);
    }
}

void AP_Doppler_Backend::parse_WE(const char *payload)
{
    const char *p = payload + 3;
    const float east_velocity_mm_s = parse_float(p);
    const float north_velocity_mm_s = parse_float(p);
    const float up_velocity_mm_s = parse_float(p);
    WaterTrackEarthVel.status = static_cast<Message_Status>(parse_char(p));
    if (WaterTrackEarthVel.status == STATUS_ACQUIRING) {
        WaterTrackEarthVel.east_velocity_mm_s = east_velocity_mm_s;
        WaterTrackEarthVel.north_velocity_mm_s = north_velocity_mm_s;
        WaterTrackEarthVel.up_velocity_mm_s = up_velocity_mm_s;
    }
}

void AP_Doppler_Backend::parse_WD(const char *payload)
{
    const char *p = payload + 3;
    WaterTrackEarthDist.east_distance_m = parse_float(p);
    WaterTrackEarthDist.north_distance_m = parse_float(p);
    WaterTrackEarthDist.up_distance_m = parse_float(p);
    WaterTrackEarthDist.center_distance_m = parse_float(p);
    WaterTrackEarthDist.time_since_valid_s = parse_float(p);
}

void AP_Doppler_Backend::parse_UA(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const Message_Status status = static_cast<Message_Status>(parse_char(p));
    update_beam_sample(_beam_samples[0], velocity_mm_s, distance_m, rssi, nsd, status);
}

void AP_Doppler_Backend::parse_UB(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const Message_Status status = static_cast<Message_Status>(parse_char(p));
    update_beam_sample(_beam_samples[1], velocity_mm_s, distance_m, rssi, nsd, status);
}

void AP_Doppler_Backend::parse_UC(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const Message_Status status = static_cast<Message_Status>(parse_char(p));
    update_beam_sample(_beam_samples[2], velocity_mm_s, distance_m, rssi, nsd, status);
}

void AP_Doppler_Backend::parse_UD(const char *payload)
{
    const char *p = payload + 3;
    const float velocity_mm_s = parse_float(p);
    const float distance_m = parse_float(p);
    const float rssi = parse_float(p);
    const float nsd = parse_float(p);
    const Message_Status status = static_cast<Message_Status>(parse_char(p));
    update_beam_sample(_beam_samples[3], velocity_mm_s, distance_m, rssi, nsd, status);
}

void AP_Doppler_Backend::parse_TD(const char *payload)
{
    const char *p = payload + 3;
    const uint32_t period_ms = parse_u32(p);
    const uint64_t echo_unix_ms = parse_u64(p);
    const uint64_t report_unix_ms = parse_u64(p);

    WITH_SEMAPHORE(_sample_sem);
    _extended_time_sample.period_ms = period_ms;
    _extended_time_sample.echo_unix_ms = echo_unix_ms;
    _extended_time_sample.report_unix_ms = report_unix_ms;
    _extended_time_sample.valid = true;
    _extended_time_sample.update_ms = AP_HAL::millis();
}

void AP_Doppler_Backend::update_velocity_sample(DVLVelocitySample &sample, float x_velocity_mm_s, float y_velocity_mm_s, float z_velocity_mm_s, DVL_LockState lock)
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

void AP_Doppler_Backend::update_beam_sample(DVLBeamSample &sample, float velocity_mm_s, float distance_m, float rssi, float nsd, Message_Status status)
{
    WITH_SEMAPHORE(_sample_sem);
    sample.status = status;
    sample.update_ms = AP_HAL::millis();
    if (status == STATUS_ACQUIRING) {
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

    if (_bottom_track_sample.valid &&
        (now_ms - _bottom_track_sample.update_ms) <= DVL_TIMEOUT_MS) {
        vel_body_mps = _bottom_track_sample.vel_body_mps;
        t_ms = _bottom_track_sample.update_ms;
        quality = _bottom_track_sample.quality;
        lock = _bottom_track_sample.lock;
        return true;
    }

    if (_water_track_sample.valid &&
        (now_ms - _water_track_sample.update_ms) <= DVL_TIMEOUT_MS) {
        vel_body_mps = _water_track_sample.vel_body_mps;
        t_ms = _water_track_sample.update_ms;
        quality = _water_track_sample.quality;
        lock = _water_track_sample.lock;
        return true;
    }

    return false;
}
