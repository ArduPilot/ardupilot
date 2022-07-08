#pragma once

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
 */

/*
 *  DCM based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_Backend.h"

class AP_AHRS_DCM : public AP_AHRS_Backend {
public:

    AP_AHRS_DCM(AP_Float &kp_yaw,
                AP_Float &kp,
                AP_Float &_gps_gain,
                AP_Float &_beta,
                AP_Enum<GPSUse> &gps_use,
                AP_Int8 &gps_minsats)
        : AP_AHRS_Backend(),
          _kp_yaw(kp_yaw),
          _kp(kp),
          gps_gain(_gps_gain),
          beta(_beta),
          _gps_use(gps_use),
          _gps_minsats(gps_minsats)
    {
        _dcm_matrix.identity();
    }

    /* Do not allow copies */
    AP_AHRS_DCM(const AP_AHRS_DCM &other) = delete;
    AP_AHRS_DCM &operator=(const AP_AHRS_DCM&) = delete;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override;

    // Methods
    void            update() override;
    void            get_results(Estimates &results) override;
    void            reset() override { reset(false); }

    // return true if yaw has been initialised
    bool yaw_initialised(void) const {
        return have_initial_yaw;
    }

    // dead-reckoning support
    virtual bool get_location(struct Location &loc) const override;

    // status reporting
    float           get_error_rp() const {
        return _error_rp;
    }
    float           get_error_yaw() const {
        return _error_yaw;
    }

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate() const override {
        return _wind;
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float &airspeed_ret) const override;

    // return an airspeed estimate if available. return true
    // if we have an estimate from a specific sensor index
    bool airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const override;

    // return a synthetic airspeed estimate (one derived from sensors
    // other than an actual airspeed sensor), if available. return
    // true if we have a synthetic airspeed.  ret will not be modified
    // on failure.
    bool synthetic_airspeed(float &ret) const override WARN_IF_UNUSED {
        ret = _last_airspeed;
        return true;
    }

    // return a ground vector estimate in meters/second, in North/East order
    Vector2f groundspeed_vector() override;

    bool            use_compass() override;

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED;

    void estimate_wind(void);

    // is the AHRS subsystem healthy?
    bool healthy() const override;

    bool get_velocity_NED(Vector3f &vec) const override;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const override;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    // relative-origin functions for fallback in AP_InertialNav
    bool get_origin(Location &ret) const override;
    bool get_relative_position_NED_origin(Vector3f &vec) const override;
    bool get_relative_position_NE_origin(Vector2f &posNE) const override;
    bool get_relative_position_D_origin(float &posD) const override;

    void send_ekf_status_report(class GCS_MAVLINK &link) const override;

private:

    // settable parameters
    AP_Float &_kp_yaw;
    AP_Float &_kp;
    AP_Float &gps_gain;

    AP_Float &beta;

    AP_Int8 &_gps_minsats;

    AP_Enum<GPSUse> &_gps_use;

    // these are experimentally derived from the simulator
    // with large drift levels
    static constexpr float _ki = 0.0087f;
    static constexpr float _ki_yaw = 0.01f;

    // accelerometer values in the earth frame in m/s/s
    Vector3f        _accel_ef[INS_MAX_INSTANCES];
    Vector3f        _accel_ef_blended;

    // Methods
    void            matrix_update(float _G_Dt);
    void            normalize(void);
    void            check_matrix(void);
    bool            renorm(Vector3f const &a, Vector3f &result);
    void            drift_correction(float deltat);
    void            drift_correction_yaw(void);
    float           yaw_error_compass(class Compass &compass);
    bool            have_gps(void) const;
    bool            use_fast_gains(void) const;
    void            backup_attitude(void);

    // internal reset function.  Called externally, we never reset the
    // DCM matrix from the eulers.  Called internally we may.
    void            reset(bool recover_eulers);

    // airspeed_ret: will always be filled-in by get_unconstrained_airspeed_estimate which fills in airspeed_ret in this order:
    //               airspeed as filled-in by an enabled airsped sensor
    //               if no airspeed sensor: airspeed estimated using the GPS speed & wind_speed_estimation
    //               Or if none of the above, fills-in using the previous airspeed estimate
    // Return false: if we are using the previous airspeed estimate
    bool get_unconstrained_airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const;

    // primary representation of attitude of board used for all inertial calculations
    Matrix3f _dcm_matrix;

    // primary representation of attitude of flight vehicle body
    Matrix3f _body_dcm_matrix;

    // euler angles - used for recovering if the DCM
    // matrix becomes ill-conditioned and watchdog storage
    float roll;
    float pitch;
    float yaw;

    Vector3f _omega_P;                          // accel Omega proportional correction
    Vector3f _omega_yaw_P;                      // proportional yaw correction
    Vector3f _omega_I;                          // Omega Integrator correction
    Vector3f _omega_I_sum;
    float _omega_I_sum_time;
    Vector3f _omega;                            // Corrected Gyro_Vector data

    bool have_initial_yaw; // true if the yaw value has been initialised with a reference

    // variables to cope with delaying the GA sum to match GPS lag
    Vector3f ra_delayed(uint8_t instance, const Vector3f &ra);
    Vector3f _ra_delay_buffer[INS_MAX_INSTANCES];

    // P term gain based on spin rate
    float           _P_gain(float spin_rate);

    // P term yaw gain based on rate of change of horiz velocity
    float           _yaw_gain(void) const;

    /* returns true if attitude should be corrected from GPS-derived
     * velocity-deltas.  We turn this off for Copter and other similar
     * vehicles while the vehicle is disarmed to avoid the HUD bobbing
     * around while the vehicle is disarmed.
     */
    bool should_correct_centrifugal() const;

    // state to support status reporting
    float _renorm_val_sum;
    uint16_t _renorm_val_count;
    float _error_rp{1.0f};
    float _error_yaw{1.0f};

    // time in microseconds of last compass update
    uint32_t _compass_last_update;

    // time in millis when we last got a GPS heading
    uint32_t _gps_last_update;

    // state of accel drift correction
    Vector3f _ra_sum[INS_MAX_INSTANCES];
    Vector3f _last_velocity;
    float _ra_deltat;
    uint32_t _ra_sum_start;

    // which accelerometer instance is active
    uint8_t _active_accel_instance;

    // the earths magnetic field
    float _last_declination;
    Vector2f _mag_earth{1, 0};

    // whether we have GPS lock
    bool _have_gps_lock;

    // the lat/lng where we last had GPS lock
    int32_t _last_lat;
    int32_t _last_lng;
    uint32_t _last_pos_ms;

    // position offset from last GPS lock
    float _position_offset_north;
    float _position_offset_east;

    // whether we have a position estimate
    bool _have_position;

    // support for wind estimation
    Vector3f _last_fuse;
    Vector3f _last_vel;
    uint32_t _last_wind_time;
    float _last_airspeed;
    uint32_t _last_consistent_heading;

    // estimated wind in m/s
    Vector3f _wind;

    float _imu1_weight{0.5f};

    // last time AHRS failed in milliseconds
    uint32_t _last_failure_ms;

    // time when DCM was last reset
    uint32_t _last_startup_ms;

    // last origin we returned, for DCM fallback from EKF
    Location last_origin;

    // Declare filter states for HPF and LPF used by complementary
    // filter in AP_AHRS::groundspeed_vector
    Vector2f _lp; // ground vector low-pass filter
    Vector2f _hp; // ground vector high-pass filter
    Vector2f _lastGndVelADS; // previous HPF input

    // pre-calculated trig cache:
    float _sin_yaw;
    float _cos_yaw;
};
