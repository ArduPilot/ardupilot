#include "AP_DAL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

enum class FrameItem : uint8_t {
    AVAILABLE_MEMORY = 0,
};

AP_DAL *AP_DAL::_singleton = nullptr;

bool AP_DAL::force_write;
bool AP_DAL::logging_started;

void AP_DAL::start_frame(AP_DAL::FrameType frametype)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)

    const AP_AHRS &ahrs = AP::ahrs();

    const uint32_t imu_us = AP::ins().get_last_update_usec();
    if (_last_imu_time_us == imu_us) {
        _RFRF.frame_types |= uint8_t(frametype);
        return;
    }
    _last_imu_time_us = imu_us;

    // we force write all msgs when logging starts
    bool logging = AP::logger().logging_started() && AP::logger().allow_start_ekf();
    if (logging && !logging_started) {
        force_write = true;
    }
    logging_started = logging;

    end_frame();

    _RFRF.frame_types = uint8_t(frametype);
    
    _RFRH.time_flying_ms = AP::vehicle()->get_time_flying_ms();
    _RFRH.time_us = AP_HAL::micros64();
    WRITE_REPLAY_BLOCK(RFRH, _RFRH);

    // update RFRN data
    const log_RFRN old = _RFRN;
    _RFRN.state_bitmask = 0;
    if (hal.util->get_soft_armed()) {
        _RFRN.state_bitmask |= uint8_t(StateMask::ARMED);
    }
    _home = ahrs.get_home();
    _RFRN.lat = _home.lat;
    _RFRN.lng = _home.lng;
    _RFRN.alt = _home.alt;
    _RFRN.get_compass_is_null = AP::ahrs().get_compass() == nullptr;
    _RFRN.rangefinder_ptr_is_null = AP::rangefinder() == nullptr;
    _RFRN.airspeed_ptr_is_null = AP::airspeed() == nullptr;
    _RFRN.EAS2TAS = AP::baro().get_EAS2TAS();
    _RFRN.vehicle_class = ahrs.get_vehicle_class();
    _RFRN.fly_forward = ahrs.get_fly_forward();
    _RFRN.ahrs_airspeed_sensor_enabled = AP::ahrs().airspeed_sensor_enabled();
    _RFRN.available_memory = hal.util->available_memory();
    WRITE_REPLAY_BLOCK_IFCHANGD(RFRN, _RFRN, old);

    // update body conversion
    _rotation_vehicle_body_to_autopilot_body = ahrs.get_rotation_vehicle_body_to_autopilot_body();

    _ins.start_frame();
    _baro.start_frame();
    _gps.start_frame();
    _compass.start_frame();
    _airspeed.start_frame();
    _rangefinder.start_frame();
    _beacon.start_frame();
    _visualodom.start_frame();

    // populate some derivative values:
    _micros = _RFRH.time_us;
    _millis = _RFRH.time_us / 1000UL;

    _trim = ahrs.get_trim();

    force_write = false;
#endif
}

void AP_DAL::end_frame(void)
{
    if (_RFRF.frame_types != 0) {
        WRITE_REPLAY_BLOCK(RFRF, _RFRF);
        _RFRF.frame_types = 0;
    }
}

void AP_DAL::log_event2(AP_DAL::Event2 event)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    end_frame();
    struct log_REV2 pkt{
        event          : uint8_t(event),
    };
    WRITE_REPLAY_BLOCK(REV2, pkt);
#endif
}

void AP_DAL::log_SetOriginLLH2(const Location &loc)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    struct log_RSO2 pkt{
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt,
    };
    WRITE_REPLAY_BLOCK(RSO2, pkt);
#endif
}

void AP_DAL::log_writeDefaultAirSpeed2(const float airspeed)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    struct log_RWA2 pkt{
        airspeed:      airspeed,
    };
    WRITE_REPLAY_BLOCK(RWA2, pkt);
#endif
}

void AP_DAL::log_event3(AP_DAL::Event3 event)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    end_frame();
    struct log_REV3 pkt{
        event          : uint8_t(event),
    };
    WRITE_REPLAY_BLOCK(REV3, pkt);
#endif
}

void AP_DAL::log_SetOriginLLH3(const Location &loc)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    struct log_RSO3 pkt{
        lat            : loc.lat,
        lng            : loc.lng,
        alt            : loc.alt,
    };
    WRITE_REPLAY_BLOCK(RSO3, pkt);
#endif
}

void AP_DAL::log_writeDefaultAirSpeed3(const float airspeed)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    struct log_RWA3 pkt{
        airspeed:      airspeed,
    };
    WRITE_REPLAY_BLOCK(RWA3, pkt);
#endif
}

void AP_DAL::log_writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    struct log_REY3 pkt{
        yawangle       : yawAngle,
        yawangleerr    : yawAngleErr,
        timestamp_ms   : timeStamp_ms,
        type           : type,
    };
    WRITE_REPLAY_BLOCK(REY3, pkt);
#endif
}

int AP_DAL::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = hal.util->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

void *AP_DAL::malloc_type(size_t size, Memory_Type mem_type)
{
    return hal.util->malloc_type(size, AP_HAL::Util::Memory_Type(mem_type));
}


const AP_DAL_Compass *AP_DAL::get_compass() const
{
    if (_RFRN.get_compass_is_null) {
        return nullptr;
    }
    return &_compass;
}

// map core number for replay
uint8_t AP_DAL::logging_core(uint8_t c) const
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    return c+100U;
#else
    return c;
#endif
}

// write out a DAL log message. If old_msg is non-null, then
// only write if the content has changed
void AP_DAL::WriteLogMessage(enum LogMessages msg_type, void *msg, const void *old_msg, uint8_t msg_size)
{
    if (!logging_started) {
        // we're not logging
        return;
    }
    // we use the _end byte to hold a flag for forcing output
    uint8_t &_end = ((uint8_t *)msg)[msg_size];
    if (old_msg && !force_write && _end == 0 && memcmp(msg, old_msg, msg_size) == 0) {
        // no change, skip this block write
        return;
    }
    if (!AP::logger().WriteReplayBlock(msg_type, msg, msg_size)) {
        // mark for forced write next time
        _end = 1;
    } else {
        _end = 0;
    }
}

#include <stdio.h>

namespace AP {

AP_DAL &dal()
{
    return *AP_DAL::get_singleton();
}

};

void xxprintf(const char *format, ...)
{
#if APM_BUILD_TYPE(APM_BUILD_Replay) || CONFIG_HAL_BOARD == HAL_BOARD_SITL
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    const char *fname = "/tmp/replay.log";
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const char *fname = "/tmp/real.log";
#endif
    static FILE *f;
    if (!f) {
        f = ::fopen(fname, "w");
    }
    va_list ap;
    va_start(ap, format);
    vfprintf(f, format, ap);
    fflush(f);
    va_end(ap);
#endif
}

