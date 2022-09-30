#pragma once

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AC_PrecLand/AC_PrecLand_StateMachine.h>

// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_Companion;
class AC_PrecLand_IRLock;
class AC_PrecLand_SITL_Gazebo;
class AC_PrecLand_SITL;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_Companion;
    friend class AC_PrecLand_IRLock;
    friend class AC_PrecLand_SITL_Gazebo;
    friend class AC_PrecLand_SITL;

public:
    AC_PrecLand();

    /* Do not allow copies */
    CLASS_NO_COPY(AC_PrecLand);

    // return singleton
    static AC_PrecLand *get_singleton() {
        return _singleton;
    }

    // perform any required initialisation of landing controllers
    // update_rate_hz should be the rate at which the update method will be called in hz
    void init(uint16_t update_rate_hz);

    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }

    // returns true if precision landing is enabled (used only for logging)
    bool enabled() const { return _enabled.get(); }

    // returns time of last update
    uint32_t last_update_ms() const { return _last_update_ms; }

    // returns time of last time target was seen
    uint32_t last_backend_los_meas_ms() const { return _last_backend_los_meas_ms; }

    // vehicle has to be closer than this many cm's to the target before descending towards target
    float get_max_xy_error_before_descending_cm() const { return _xy_max_dist_desc * 100.0f; }

    // returns orientation of sensor
    Rotation get_orient() const { return _orient; }

    // returns ekf outlier count
    uint32_t ekf_outlier_count() const { return _outlier_reject_count; }

    // give chance to driver to get updates from sensor, should be called at 400hz
    void update(float rangefinder_alt_cm, bool rangefinder_alt_valid);

    // returns target position relative to the EKF origin
    bool get_target_position_cm(Vector2f& ret);

    // returns target relative position as 3D vector
    void get_target_position_measurement_cm(Vector3f& ret);

    // returns target position relative to vehicle
    bool get_target_position_relative_cm(Vector2f& ret);

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative_cms(Vector2f& ret);

    // get the absolute velocity of the vehicle
    void get_target_velocity_cms(const Vector2f& vehicle_velocity_cms, Vector2f& target_vel_cms);

    // returns true when the landing target has been detected
    bool target_acquired();

    // process a LANDING_TARGET mavlink message
    void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms);

    // State of the Landing Target Location
    enum class TargetState: uint8_t {
        TARGET_NEVER_SEEN = 0,
        TARGET_OUT_OF_RANGE,
        TARGET_RECENTLY_LOST,
        TARGET_FOUND
    };

    // return the last time PrecLand library had a output of the landing target position
    uint32_t get_last_valid_target_ms() const { return _last_valid_target_ms; }

    // return the current state of the location of the target
    TargetState get_target_state() const { return _current_target_state; }

    // return the last known landing position in Earth Frame NED meters.
    void get_last_detected_landing_pos(Vector3f &pos) const { pos = _last_target_pos_rel_origin_NED; }

    // return the last known postion of the vehicle when the target was detected in Earth Frame NED meters.
    void get_last_vehicle_pos_when_target_detected(Vector3f &pos) const { pos = _last_vehicle_pos_NED; }

    // Parameter getters
    AC_PrecLand_StateMachine::RetryStrictness get_retry_strictness() const { return static_cast<AC_PrecLand_StateMachine::RetryStrictness>(_strict.get()); }
    uint8_t get_max_retry_allowed() const { return _retry_max; }
    float get_min_retry_time_sec() const { return _retry_timeout_sec; }
    AC_PrecLand_StateMachine::RetryAction get_retry_behaviour() const { return static_cast<AC_PrecLand_StateMachine::RetryAction>(_retry_behave.get()); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class EstimatorType : uint8_t {
        RAW_SENSOR = 0,
        KALMAN_FILTER = 1,
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum class Type : uint8_t {
        NONE = 0,
        COMPANION = 1,
        IRLOCK = 2,
        SITL_GAZEBO = 3,
        SITL = 4,
    };

    enum PLndOptions {
        PLND_OPTION_DISABLED = 0,
        PLND_OPTION_MOVING_TARGET = (1 << 0),
    };

    // check the status of the target
    void check_target_status(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // Check if the landing target is supposed to be in sight based on the height of the vehicle from the ground
    // This needs a valid rangefinder to work, if the min/max parameters are non zero
    bool check_if_sensor_in_range(float rangefinder_alt_m, bool rangefinder_alt_valid) const;

    // check if EKF got the time to initialize when the landing target was first detected
    // Expects sensor to update within EKF_INIT_SENSOR_MIN_UPDATE_MS milliseconds till EKF_INIT_TIME_MS milliseconds have passed
    // after this period landing target estimates can be used by vehicle
    void check_ekf_init_timeout();

    // run target position estimator
    void run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // If a new measurement was retrieved, sets _target_pos_rel_meas_NED and returns true
    bool construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // get vehicle body frame 3D vector from vehicle to target.  returns true on success, false on failure
    bool retrieve_los_meas(Vector3f& target_vec_unit_body);

    // calculate target's position and velocity relative to the vehicle (used as input to position controller)
    // results are stored in_target_pos_rel_out_NE, _target_vel_rel_out_NE
    void run_output_prediction();

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled
    AP_Enum<Type>               _type;              // precision landing sensor type
    AP_Int8                     _bus;               // which sensor bus
    AP_Enum<EstimatorType>      _estimator_type;    // precision landing estimator type
    AP_Float                    _lag;               // sensor lag in seconds
    AP_Float                    _yaw_align;         // Yaw angle from body x-axis to sensor x-axis.
    AP_Float                    _land_ofs_cm_x;     // Desired landing position of the camera forward of the target in vehicle body frame
    AP_Float                    _land_ofs_cm_y;     // Desired landing position of the camera right of the target in vehicle body frame
    AP_Float                    _accel_noise;       // accelerometer process noise
    AP_Vector3f                 _cam_offset;        // Position of the camera relative to the CG
    AP_Float                    _xy_max_dist_desc;  // Vehicle doing prec land will only descent vertically when horizontal error (in m) is below this limit
    AP_Int8                     _strict;            // PrecLand strictness
    AP_Int8                     _retry_max;         // PrecLand Maximum number of retires to a failed landing
    AP_Float                    _retry_timeout_sec; // Time for which vehicle continues descend even if target is lost. After this time period, vehicle will attemp a landing retry depending on PLND_STRICT param.
    AP_Int8                     _retry_behave;      // Action to do when trying a landing retry
    AP_Float                    _sensor_min_alt;     // PrecLand minimum height required for detecting target
    AP_Float                    _sensor_max_alt;     // PrecLand maximum height the sensor can detect target
    AP_Int16                    _options;            // Bitmask for extra options
    AP_Enum<Rotation>           _orient;             // Orientation of camera/sensor

    uint32_t                    _last_update_ms;    // system time in millisecond when update was last called
    bool                        _target_acquired;   // true if target has been seen recently after estimator is initialized
    bool                        _estimator_initialized; // true if estimator has been initialized after few seconds of the target being detected by sensor
    uint32_t                    _estimator_init_ms; // system time in millisecond when EKF was init
    uint32_t                    _last_backend_los_meas_ms;  // system time target was last seen
    uint32_t                    _last_valid_target_ms;       // last time PrecLand library had a output of the landing target position

    PosVelEKF                   _ekf_x, _ekf_y;     // Kalman Filter for x and y axis
    uint32_t                    _outlier_reject_count;  // mini-EKF's outlier counter (3 consecutive outliers lead to EKF accepting updates)

    Vector3f                    _target_pos_rel_meas_NED; // target's relative position as 3D vector
    Vector3f                    _approach_vector_body;   // unit vector in landing approach direction (in body frame)

    Vector3f                    _last_target_pos_rel_origin_NED;  // stores the last known location of the target horizontally, and the height of the vehicle where it detected this target in meters NED
    Vector3f                    _last_vehicle_pos_NED;            // stores the position of the vehicle when landing target was last detected in m and NED
    Vector2f                    _target_pos_rel_est_NE; // target's position relative to the IMU, not compensated for lag
    Vector2f                    _target_vel_rel_est_NE; // target's velocity relative to the IMU, not compensated for lag

    Vector2f                    _target_pos_rel_out_NE; // target's position relative to the camera, fed into position controller
    Vector2f                    _target_vel_rel_out_NE; // target's velocity relative to the CG, fed into position controller

    TargetState                 _current_target_state;  // Current status of the landing target

    // structure and buffer to hold a history of vehicle velocity
    struct inertial_data_frame_s {
        Matrix3f Tbn;                               // dcm rotation matrix to rotate body frame to north
        Vector3f correctedVehicleDeltaVelocityNED;
        Vector3f inertialNavVelocity;
        bool inertialNavVelocityValid;
        float dt;
        uint64_t time_usec;
    };
    ObjectArray<inertial_data_frame_s> *_inertial_history;

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver

    // write out PREC message to log:
    void Write_Precland();
    uint32_t last_log_ms;  // last time we logged

    static AC_PrecLand *_singleton; //singleton
};

namespace AP {
    AC_PrecLand *ac_precland();
};
