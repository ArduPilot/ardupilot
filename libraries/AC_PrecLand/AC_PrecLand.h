#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_Buffer/AP_Buffer.h>
#include <AP_AHRS/AP_AHRS.h>

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
    AC_PrecLand(const AP_AHRS_NavEKF& ahrs);

    /* Do not allow copies */
    AC_PrecLand(const AC_PrecLand &other) = delete;
    AC_PrecLand &operator=(const AC_PrecLand&) = delete;

    // precision landing behaviours (held in PRECLAND_ENABLED parameter)
    enum PrecLandBehaviour {
        PRECLAND_BEHAVIOUR_DISABLED,
        PRECLAND_BEHAVIOR_ALWAYSLAND,
        PRECLAND_BEHAVIOR_CAUTIOUS
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum PrecLandType {
        PRECLAND_TYPE_NONE = 0,
        PRECLAND_TYPE_COMPANION,
        PRECLAND_TYPE_IRLOCK,
        PRECLAND_TYPE_SITL_GAZEBO,
        PRECLAND_TYPE_SITL,
    };

    // perform any required initialisation of landing controllers
    void init();

    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }

    // returns true if precision landing is enabled (used only for logging)
    bool enabled() const { return _enabled.get(); }

    // returns time of last update
    uint32_t last_update_ms() const { return _last_update_ms; }

    // give chance to driver to get updates from sensor, should be called at 400hz
    void update(float rangefinder_alt_cm, bool rangefinder_alt_valid);

    // returns target position relative to the EKF origin
    bool get_target_position_cm(Vector2f& ret);

    // returns target position relative to vehicle
    bool get_target_position_relative_cm(Vector2f& ret);

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative_cms(Vector2f& ret);

    // returns true when the landing target has been detected
    bool target_acquired();

    // process a LANDING_TARGET mavlink message
    void handle_msg(mavlink_message_t* msg);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    enum estimator_type_t {
        ESTIMATOR_TYPE_RAW_SENSOR = 0,
        ESTIMATOR_TYPE_KALMAN_FILTER = 1
    };

    // returns enabled parameter as an behaviour
    enum PrecLandBehaviour get_behaviour() const { return (enum PrecLandBehaviour)(_enabled.get()); }

    // run target position estimator
    void run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // If a new measurement was retreived, sets _target_pos_rel_meas_NED and returns true
    bool construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // get vehicle body frame 3D vector from vehicle to target.  returns true on success, false on failure
    bool retrieve_los_meas(Vector3f& target_vec_unit_body);

    // calculate target's position and velocity relative to the vehicle (used as input to position controller)
    // results are stored in_target_pos_rel_out_NE, _target_vel_rel_out_NE
    void run_output_prediction();

    // references to inertial nav and ahrs libraries
    const AP_AHRS_NavEKF&       _ahrs;

    // parameters
    AP_Int8                     _enabled;           // enabled/disabled and behaviour
    AP_Int8                     _type;              // precision landing sensor type
    AP_Int8                     _bus;               // which sensor bus
    AP_Int8                     _estimator_type;    // precision landing estimator type
    AP_Float                    _yaw_align;         // Yaw angle from body x-axis to sensor x-axis.
    AP_Float                    _land_ofs_cm_x;     // Desired landing position of the camera forward of the target in vehicle body frame
    AP_Float                    _land_ofs_cm_y;     // Desired landing position of the camera right of the target in vehicle body frame
    AP_Float                    _accel_noise;       // accelometer process noise
    AP_Vector3f                 _cam_offset;        // Position of the camera relative to the CG

    uint32_t                    _last_update_ms;    // system time in millisecond when update was last called
    bool                        _target_acquired;   // true if target has been seen recently
    uint32_t                    _last_backend_los_meas_ms;  // system time target was last seen

    PosVelEKF                   _ekf_x, _ekf_y;     // Kalman Filter for x and y axis
    uint32_t                    _outlier_reject_count;  // mini-EKF's outlier counter (3 consecutive outliers lead to EKF accepting updates)
    
    Vector3f                    _target_pos_rel_meas_NED; // target's relative position as 3D vector

    Vector2f                    _target_pos_rel_est_NE; // target's position relative to the IMU, not compensated for lag
    Vector2f                    _target_vel_rel_est_NE; // target's velocity relative to the IMU, not compensated for lag

    Vector2f                    _target_pos_rel_out_NE; // target's position relative to the camera, fed into position controller
    Vector2f                    _target_vel_rel_out_NE; // target's velocity relative to the CG, fed into position controller

    // structure and buffer to hold a short history of vehicle velocity
    struct inertial_data_frame_s {
        Matrix3f Tbn;                               // dcm rotation matrix to rotate body frame to north
        Vector3f correctedVehicleDeltaVelocityNED;
        Vector3f inertialNavVelocity;
        bool inertialNavVelocityValid;
        float dt;
    };
    AP_Buffer<inertial_data_frame_s,8>       _inertial_history;

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver
};
