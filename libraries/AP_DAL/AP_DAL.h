#pragma once

#include "AP_DAL_InertialSensor.h"
#include "AP_DAL_Baro.h"
#include "AP_DAL_GPS.h"
#include "AP_DAL_RangeFinder.h"
#include "AP_DAL_Compass.h"
#include "AP_DAL_Airspeed.h"
#include "AP_DAL_Beacon.h"
#include "AP_DAL_VisualOdom.h"

#include "LogStructure.h"

#include <stdio.h>
#include <stdint.h>
#include <cstddef>


#define DAL_CORE(c) AP::dal().logging_core(c)

class NavEKF2;
class NavEKF3;

class AP_DAL {
public:

    enum class FrameType : uint8_t {
        InitialiseFilterEKF2 = 1U<<0,
        UpdateFilterEKF2 = 1U<<1,
        InitialiseFilterEKF3 = 1<<2,
        UpdateFilterEKF3 = 1<<3,
        LogWriteEKF2 = 1<<4,
        LogWriteEKF3 = 1<<5,
    };

    enum class Event : uint8_t {
        resetGyroBias             =  0,
        resetHeightDatum          =  1,
        //setInhibitGPS           =  2, // removed
        //setTakeoffExpected        =  3, // removed
        //unsetTakeoffExpected      =  4, // removed
        //setTouchdownExpected      =  5, // removed
        //unsetTouchdownExpected    =  6, // removed
        //setInhibitGpsVertVelUse   =  7, // removed
        //unsetInhibitGpsVertVelUse =  8, // removed
        setTerrainHgtStable       =  9,
        unsetTerrainHgtStable     = 10,
        requestYawReset           = 11,
        checkLaneSwitch           = 12,
        setSourceSet0             = 13,
        setSourceSet1             = 14,
        setSourceSet2             = 15,
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
    uint64_t micros64() const { return _RFRH.time_us; }
    uint32_t micros() const { return _micros; }
    uint32_t millis() const { return _millis; }

    void log_event2(Event event);
    void log_SetOriginLLH2(const Location &loc);
    void log_writeDefaultAirSpeed2(const float aspeed, const float uncertainty);

    void log_event3(Event event);
    void log_SetOriginLLH3(const Location &loc);
    void log_writeDefaultAirSpeed3(const float aspeed, const float uncertainty);
    void log_writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type);

    enum class StateMask {
        ARMED = (1U<<0),
    };

    // EKF ID for timing checks
    enum class EKFType : uint8_t {
        EKF2 = 0,
        EKF3 = 1,
    };

    // check if we are low on CPU for this core
    bool ekf_low_time_remaining(EKFType etype, uint8_t core);
    
    // returns armed state for the current frame
    bool get_armed() const { return _RFRN.armed; }

    // memory available at start of current frame.  While this could
    // potentially change as we go through the frame, the
    // ramifications of being out of memory are that you don't start
    // the EKF, so the simplicity of having one value for the entire
    // frame is worthwhile.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    uint32_t available_memory() const { return _RFRN.available_memory + 10240; }
#else
    uint32_t available_memory() const { return _RFRN.available_memory; }
#endif

    int8_t get_ekf_type(void) const {
        return _RFRN.ekf_type;
    }

    int snprintf(char* str, size_t size, const char *format, ...) const;

    // copied in AP_HAL/Util.h
    enum Memory_Type {
        MEM_DMA_SAFE,
        MEM_FAST
    };
    void *malloc_type(size_t size, enum Memory_Type mem_type) const;

    AP_DAL_InertialSensor &ins() { return _ins; }
    AP_DAL_Baro &baro() { return _baro; }
    AP_DAL_GPS &gps() { return _gps; }

    AP_DAL_RangeFinder *rangefinder() {
        return _rangefinder;
    }
    AP_DAL_Airspeed *airspeed() {
        return _airspeed;
    }
#if AP_BEACON_ENABLED
    AP_DAL_Beacon *beacon() {
        return _beacon;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    AP_DAL_VisualOdom *visualodom() {
        return _visualodom;
    }
#endif

    AP_DAL_Compass &compass() { return _compass; }

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

    bool get_takeoff_expected(void) const {
        return _RFRN.takeoff_expected;
    }

    bool get_touchdown_expected(void) const {
        return _RFRN.touchdown_expected;
    }

    // for EKF usage to enable takeoff expected to true
    void set_takeoff_expected();

    // get ahrs trim
    const Vector3f &get_trim() const {
        return _RFRN.ahrs_trim;
    }

    const Matrix3f &get_rotation_vehicle_body_to_autopilot_body(void) const {
        return _rotation_vehicle_body_to_autopilot_body;
    }

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change
    const class Location &get_home(void) const {
        return _home;
    }

    uint32_t get_time_flying_ms(void) const {
        return _RFRH.time_flying_ms;
    }

    bool opticalflow_enabled(void) const {
        return _RFRN.opticalflow_enabled;
    }

    bool wheelencoder_enabled(void) const {
        return _RFRN.wheelencoder_enabled;
    }

    // log optical flow data
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride);

    // log external nav data
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    // log wheel odomotry data
    void writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius);
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset);

    // Replay support:
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
    void handle_message(const log_RFRF &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);

    void handle_message(const log_RISH &msg) {
        _ins.handle_message(msg);
    }
    void handle_message(const log_RISI &msg) {
        _ins.handle_message(msg);
    }

    void handle_message(const log_RASH &msg) {
        if (_airspeed == nullptr) {
            _airspeed = new AP_DAL_Airspeed;
        }
        _airspeed->handle_message(msg);
    }
    void handle_message(const log_RASI &msg) {
        if (_airspeed == nullptr) {
            _airspeed = new AP_DAL_Airspeed;
        }
        _airspeed->handle_message(msg);
    }

    void handle_message(const log_RBRH &msg) {
        _baro.handle_message(msg);
    }
    void handle_message(const log_RBRI &msg) {
        _baro.handle_message(msg);
    }

    void handle_message(const log_RRNH &msg) {
        if (_rangefinder == nullptr) {
            _rangefinder = new AP_DAL_RangeFinder;
        }
        _rangefinder->handle_message(msg);
    }
    void handle_message(const log_RRNI &msg) {
        if (_rangefinder == nullptr) {
            _rangefinder = new AP_DAL_RangeFinder;
        }
        _rangefinder->handle_message(msg);
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
#if AP_BEACON_ENABLED
        if (_beacon == nullptr) {
            _beacon = new AP_DAL_Beacon;
        }
        _beacon->handle_message(msg);
#endif
    }
    void handle_message(const log_RBCI &msg) {
#if AP_BEACON_ENABLED
        if (_beacon == nullptr) {
            _beacon = new AP_DAL_Beacon;
        }
        _beacon->handle_message(msg);
#endif
    }
    void handle_message(const log_RVOH &msg) {
#if HAL_VISUALODOM_ENABLED
        if (_visualodom == nullptr) {
            _visualodom = new AP_DAL_VisualOdom;
        }
        _visualodom->handle_message(msg);
#endif
    }
    void handle_message(const log_ROFH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);
    void handle_message(const log_REPH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);
    void handle_message(const log_REVH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);
    void handle_message(const log_RWOH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);
    void handle_message(const log_RBOH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);

    // map core number for replay
    uint8_t logging_core(uint8_t c) const;

    // write out a DAL log message. If old_msg is non-null, then
    // only write if the content has changed
    static void WriteLogMessage(enum LogMessages msg_type, void *msg, const void *old_msg, uint8_t msg_size);

private:

    static AP_DAL *_singleton;

    // framing structures
    struct log_RFRH _RFRH;
    struct log_RFRF _RFRF;
    struct log_RFRN _RFRN;

    // push-based sensor structures
    struct log_ROFH _ROFH;
    struct log_REPH _REPH;
    struct log_REVH _REVH;
    struct log_RWOH _RWOH;
    struct log_RBOH _RBOH;

    // cached variables for speed:
    uint32_t _micros;
    uint32_t _millis;

    Matrix3f _rotation_vehicle_body_to_autopilot_body;
    Location _home;
    uint32_t _last_imu_time_us;

    AP_DAL_InertialSensor _ins;
    AP_DAL_Baro _baro;
    AP_DAL_GPS _gps;
    AP_DAL_RangeFinder *_rangefinder;
    AP_DAL_Compass _compass;
    AP_DAL_Airspeed *_airspeed;
#if AP_BEACON_ENABLED
    AP_DAL_Beacon *_beacon;
#endif
#if HAL_VISUALODOM_ENABLED
    AP_DAL_VisualOdom *_visualodom;
#endif

    static bool logging_started;
    static bool force_write;

    bool ekf2_init_done;
    bool ekf3_init_done;

    void init_sensors(void);
    bool init_done;
};

#define WRITE_REPLAY_BLOCK(sname,v) AP_DAL::WriteLogMessage(LOG_## sname ##_MSG, &v, nullptr, offsetof(log_ ##sname, _end))
#define WRITE_REPLAY_BLOCK_IFCHANGED(sname,v,old) do { static_assert(sizeof(v) == sizeof(old), "types must match"); \
                                                      AP_DAL::WriteLogMessage(LOG_## sname ##_MSG, &v, &old, offsetof(log_ ##sname, _end)); } \
                                                 while (0)

namespace AP {
    AP_DAL &dal();
};

// replay printf for debugging
void rprintf(const char *format, ...);

