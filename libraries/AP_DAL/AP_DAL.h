#pragma once

#include <stdint.h>

#include <cstddef>

#include "AP_DAL_InertialSensor.h"
#include "AP_DAL_Baro.h"
#include "AP_DAL_GPS.h"
#include "AP_DAL_RangeFinder.h"
#include "AP_DAL_Compass.h"
#include "AP_DAL_Airspeed.h"
#include "AP_DAL_Beacon.h"
#include "AP_DAL_VisualOdom.h"

#include "LogStructure.h"

#define DAL_CORE(c) AP::dal().logging_core(c)

class AP_DAL {
public:

    enum class FrameType {
        InitialiseFilterEKF2 = 1<<0,
        UpdateFilterEKF2 = 1<<1,
        InitialiseFilterEKF3 = 1<<2,
        UpdateFilterEKF3 = 1<<3,
        LogWriteEKF2 = 1<<4,
        LogWriteEKF3 = 1<<5,
    };

    enum class Event2 {
        resetGyroBias             =  0,
        resetHeightDatum          =  1,
        setInhibitGPS             =  2,
        setTakeoffExpected        =  3,
        unsetTakeoffExpected      =  4,
        setTouchdownExpected      =  5,
        unsetTouchdownExpected    =  6,
        setInhibitGpsVertVelUse   =  7,
        unsetInhibitGpsVertVelUse =  8,
        setTerrainHgtStable       =  9,
        unsetTerrainHgtStable     = 10,
        requestYawReset           = 11,
        checkLaneSwitch           = 12,
    };

    enum class Event3 {
        resetGyroBias             =  0,
        resetHeightDatum          =  1,
        setInhibitGPS             =  2,
        setTakeoffExpected        =  3,
        unsetTakeoffExpected      =  4,
        setTouchdownExpected      =  5,
        unsetTouchdownExpected    =  6,
        setInhibitGpsVertVelUse   =  7,
        unsetInhibitGpsVertVelUse =  8,
        setTerrainHgtStable       =  9,
        unsetTerrainHgtStable     = 10,
        requestYawReset           = 11,
        checkLaneSwitch           = 12,
    };

    // must remain the same as AP_AHRS_VehicleClass numbers-wise
    enum class VehicleClass : uint8_t {
        UNKNOWN,
        GROUND,
        COPTER,
        FIXED_WING,
        SUBMARINE,
    };

    AP_DAL() {}

    static AP_DAL *get_singleton() {
        if (!_singleton) {
            _singleton = new AP_DAL();
        }
        return _singleton;
    }

    void start_frame(FrameType frametype);
    void end_frame(void);
    uint64_t micros64() { return _RFRH.time_us; }
    uint32_t micros() { return _micros; }
    uint32_t millis() { return _millis; }

    void log_event2(Event2 event);
    void log_SetOriginLLH2(const Location &loc);
    void log_writeDefaultAirSpeed2(float airspeed);

    void log_event3(Event3 event);
    void log_SetOriginLLH3(const Location &loc);
    void log_writeDefaultAirSpeed3(float airspeed);
    void log_writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type);

    enum class StateMask {
        ARMED = (1U<<0),
    };

    // returns armed state for the current frame
    bool get_armed() { return _RFRN.state_bitmask & uint8_t(StateMask::ARMED); }

    // memory available at start of current frame.  While this could
    // potentially change as we go through the frame, the
    // ramifications of being out of memory are that you don't start
    // the EKF, so the simplicity of having one value for the entire
    // frame is worthwhile.
    uint32_t available_memory() { return _RFRN.available_memory; }

    int8_t get_ekf_type(void) const {
        return _RFRN.ekf_type;
    }

    int snprintf(char* str, size_t size, const char *format, ...);

    // copied in AP_HAL/Util.h
    enum Memory_Type {
        MEM_DMA_SAFE,
        MEM_FAST
    };
    void *malloc_type(size_t size, enum Memory_Type mem_type);

    AP_DAL_InertialSensor &ins() { return _ins; }
    AP_DAL_Baro &baro() { return _baro; }
    AP_DAL_GPS &gps() { return _gps; }
    AP_DAL_RangeFinder *rangefinder() {
        if (_RFRN.rangefinder_ptr_is_null) {
            return nullptr;
        }
        return &_rangefinder;
    }
    AP_DAL_Airspeed *airspeed() {
        if (_RFRN.airspeed_ptr_is_null) {
            return nullptr;
        }
        return &_airspeed;
    }
    AP_DAL_Beacon *beacon() {
        return _beacon.beacon();
    }
#if HAL_VISUALODOM_ENABLED
    AP_DAL_VisualOdom *visualodom() {
        return _visualodom.visualodom();
    }
#endif

    // this method *always* returns you the compass.  This is in
    // constrast to get_compass, which only returns the compass once
    // the vehicle deigns to permit its use by the EKF.
    AP_DAL_Compass &compass() { return _compass; }

    // this call replaces AP::ahrs()->get_compass(), whose return
    // result can be varied by the vehicle (typically by setting when
    // first reading is received).  This is explicitly not
    // "AP_DAL_Compass &compass() { return _compass; } - but it should
    // change to be that.
    const AP_DAL_Compass *get_compass() const;

    // random methods that AP_NavEKF3 wants to call on AHRS:
    bool airspeed_sensor_enabled(void) const {
        return _RFRN.ahrs_airspeed_sensor_enabled;
    }

    // this replaces AP::ahrs()->EAS2TAS(), which should probably go
    // away in favour of just using the Baro method.
    // get apparent to true airspeed ratio
    float get_EAS2TAS(void) const {
        return _RFRN.EAS2TAS;
    }

    VehicleClass get_vehicle_class(void) const {
        return (VehicleClass)_RFRN.vehicle_class;
    }

    bool get_fly_forward(void) const {
        return _RFRN.fly_forward;
    }

    // get trim
    const Vector3f &get_trim() const {
        return _trim;
    }

    const Matrix3f &get_rotation_vehicle_body_to_autopilot_body(void) const {
        return _rotation_vehicle_body_to_autopilot_body;
    }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change
    const struct Location &get_home(void) const {
        return _home;
    }

    uint32_t get_time_flying_ms(void) const {
        return _RFRH.time_flying_ms;
    }

    // Replay support:
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    void handle_message(const log_RFRH &msg) {
        _RFRH = msg;
        _micros = _RFRH.time_us;
        _millis = _RFRH.time_us / 1000UL;
    }
    void handle_message(const log_RFRN &msg) {
        _RFRN = msg;
        _home.lat = msg.lat;
        _home.lng = msg.lng;
        _home.alt = msg.alt;
    }

    void handle_message(const log_RISH &msg) {
        _ins.handle_message(msg);
    }
    void handle_message(const log_RISI &msg) {
        _ins.handle_message(msg);
    }

    void handle_message(const log_RASH &msg) {
        _airspeed.handle_message(msg);
    }
    void handle_message(const log_RASI &msg) {
        _airspeed.handle_message(msg);
    }

    void handle_message(const log_RBRH &msg) {
        _baro.handle_message(msg);
    }
    void handle_message(const log_RBRI &msg) {
        _baro.handle_message(msg);
    }

    void handle_message(const log_RRNH &msg) {
        _rangefinder.handle_message(msg);
    }
    void handle_message(const log_RRNI &msg) {
        _rangefinder.handle_message(msg);
    }

    void handle_message(const log_RGPH &msg) {
        _gps.handle_message(msg);
    }
    void handle_message(const log_RGPI &msg) {
        _gps.handle_message(msg);
    }
    void handle_message(const log_RGPJ &msg) {
        _gps.handle_message(msg);
    }

    void handle_message(const log_RMGH &msg) {
        _compass.handle_message(msg);
    }
    void handle_message(const log_RMGI &msg) {
        _compass.handle_message(msg);
    }

    void handle_message(const log_RBCH &msg) {
        _beacon.handle_message(msg);
    }
    void handle_message(const log_RBCI &msg) {
        _beacon.handle_message(msg);
    }
    void handle_message(const log_RVOH &msg) {
#if HAL_VISUALODOM_ENABLED
        _visualodom.handle_message(msg);
#endif
    }
#endif

    // map core number for replay
    uint8_t logging_core(uint8_t c) const;

    // write out a DAL log message. If old_msg is non-null, then
    // only write if the content has changed
    static void WriteLogMessage(enum LogMessages msg_type, void *msg, const void *old_msg, uint8_t msg_size);

private:

    static AP_DAL *_singleton;

    struct log_RFRH _RFRH;
    struct log_RFRF _RFRF;
    struct log_RFRN _RFRN;

    // cached variables for speed:
    uint32_t _micros;
    uint32_t _millis;

    Vector3f _trim;
    Matrix3f _rotation_vehicle_body_to_autopilot_body;
    Location _home;
    uint32_t _last_imu_time_us;

    AP_DAL_InertialSensor _ins;
    AP_DAL_Baro _baro;
    AP_DAL_GPS _gps;
    AP_DAL_RangeFinder _rangefinder;
    AP_DAL_Compass _compass;
    AP_DAL_Airspeed _airspeed;
    AP_DAL_Beacon _beacon;
#if HAL_VISUALODOM_ENABLED
    AP_DAL_VisualOdom _visualodom;
#endif

    static bool logging_started;
    static bool force_write;
};

#define WRITE_REPLAY_BLOCK(sname,v) AP_DAL::WriteLogMessage(LOG_## sname ##_MSG, &v, nullptr, offsetof(log_ ##sname, _end))
#define WRITE_REPLAY_BLOCK_IFCHANGD(sname,v,old) AP_DAL::WriteLogMessage(LOG_## sname ##_MSG, &v, &old, offsetof(log_ ##sname, _end))

namespace AP {
    AP_DAL &dal();
};

void xxprintf(const char *format, ...);

