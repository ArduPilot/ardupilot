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
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include <AP_HAL/AP_HAL.h>

#define AP_AHRS_NAVEKF_AVAILABLE 1
#include "AP_AHRS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters


#define AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000     // time in milliseconds the ekf needs to settle after being started

class AP_AHRS_NavEKF : public AP_AHRS_DCM {
public:
    enum Flags {
        FLAG_NONE = 0,
        FLAG_ALWAYS_USE_EKF = 0x1,
    };

    // Constructor
    AP_AHRS_NavEKF(NavEKF2 &_EKF2, NavEKF3 &_EKF3, Flags flags = FLAG_NONE);

    /* Do not allow copies */
    AP_AHRS_NavEKF(const AP_AHRS_NavEKF &other) = delete;
    AP_AHRS_NavEKF &operator=(const AP_AHRS_NavEKF&) = delete;

    // return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const override;
    const Matrix3f &get_rotation_body_to_ned(void) const override;

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const override;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override;

    void            update(bool skip_ins_update=false) override;
    void            reset(bool recover_eulers = false) override;

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw) override;

    // dead-reckoning support
    bool get_position(struct Location &loc) const override;

    // get latest altitude estimate above ground level in meters and validity flag
    bool get_hagl(float &hagl) const override;

    // status reporting of estimated error
    float           get_error_rp() const override;
    float           get_error_yaw() const override;

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate() const override;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret) const override;

    // true if compass is being used
    bool use_compass() override;

    // we will need to remove these to fully hide which EKF we are using
    NavEKF2 &get_NavEKF2(void) {
        return EKF2;
    }
    const NavEKF2 &get_NavEKF2_const(void) const {
        return EKF2;
    }

    NavEKF3 &get_NavEKF3(void) {
        return EKF3;
    }
    const NavEKF3 &get_NavEKF3_const(void) const {
        return EKF3;
    }
    
    // return secondary attitude solution if available, as eulers in radians
    bool get_secondary_attitude(Vector3f &eulers) const override;

    // return secondary attitude solution if available, as quaternion
    bool get_secondary_quaternion(Quaternion &quat) const override;
    
    // return secondary position solution if available
    bool get_secondary_position(struct Location &loc) const override;

    // EKF has a better ground speed vector estimate
    Vector2f groundspeed_vector() override;

    const Vector3f &get_accel_ef(uint8_t i) const override;
    const Vector3f &get_accel_ef() const override;

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const override;

    // blended accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef_blended() const override;

    // set the EKF's origin location in 10e7 degrees.  This should only
    // be called when the EKF has no absolute position reference (i.e. GPS)
    // from which to decide the origin on its own
    bool set_origin(const Location &loc) override;

    // returns the inertial navigation origin in lat/lon/alt
    bool get_origin(Location &ret) const override;

    bool have_inertial_nav() const override;

    bool get_velocity_NED(Vector3f &vec) const override;

    // return the relative position NED to either home or origin
    // return true if the estimate is valid
    bool get_relative_position_NED_home(Vector3f &vec) const override;
    bool get_relative_position_NED_origin(Vector3f &vec) const override;

    // return the relative position NE to either home or origin
    // return true if the estimate is valid
    bool get_relative_position_NE_home(Vector2f &posNE) const override;
    bool get_relative_position_NE_origin(Vector2f &posNE) const override;

    // return the relative position down to either home or origin
    // baro will be used for the _home relative one if the EKF isn't
    void get_relative_position_D_home(float &posD) const override;
    bool get_relative_position_D_origin(float &posD) const override;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const;

    // write optical flow measurements to EKF
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset);

    // write body odometry measurements to the EKF
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset);

    // Write position and quaternion data from an external navigation system
    void writeExtNavData(const Vector3f &sensOffset, const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint32_t resetTime_ms) override;

    // inhibit GPS usage
    uint8_t setInhibitGPS(void);

    // get speed limit
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    void set_ekf_use(bool setting);

    // is the AHRS subsystem healthy?
    bool healthy() const override;

    // true if the AHRS has completed initialisation
    bool initialised() const override;

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const;

    // get compass offset estimates
    // true if offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    // report any reason for why the backend is refusing to initialise
    const char *prearm_failure_reason(void) const override;

    // check all cores providing consistent attitudes for prearm checks
    bool attitudes_consistent() const override;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng) const override;

    // return the amount of NE position change in meters due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const override;

    // return the amount of NE velocity change in meters/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const override;

    // return the amount of vertical position change due to the last reset in meters
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosDownReset(float &posDelta) const override;

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum() override;

    // send a EKF_STATUS_REPORT for current EKF
    void send_ekf_status_report(mavlink_channel_t chan) const;
    
    // get_hgt_ctrl_limit - get maximum height to be observed by the control loops in meters and a validity flag
    // this is used to limit height during optical flow navigation
    // it will return invalid when no limiting is required
    bool get_hgt_ctrl_limit(float &limit) const override;

    // get_location - updates the provided location with the latest
    // calculated location including absolute altitude
    // returns true on success (i.e. the EKF knows it's latest
    // position), false on failure
    bool get_location(struct Location &loc) const;

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const override;

    // returns the expected NED magnetic field
    bool get_mag_field_NED(Vector3f& ret) const;

    // returns the estimated magnetic field offsets in body frame
    bool get_mag_field_correction(Vector3f &ret) const override;

    void setTakeoffExpected(bool val);
    void setTouchdownExpected(bool val);

    bool getGpsGlitchStatus() const;

    // used by Replay to force start at right timestamp
    void force_ekf_start(void) { _force_ekf = true; }

    // is the EKF backend doing its own sensor logging?
    bool have_ekf_logging(void) const override;

    // get the index of the current primary accelerometer sensor
    uint8_t get_primary_accel_index(void) const override;

    // get the index of the current primary gyro sensor
    uint8_t get_primary_gyro_index(void) const override;

private:
    enum EKF_TYPE {EKF_TYPE_NONE=0,
                   EKF_TYPE3=3,
                   EKF_TYPE2=2
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                   ,EKF_TYPE_SITL=10
#endif
    };
    EKF_TYPE active_EKF_type(void) const;

    bool always_use_EKF() const {
        return _ekf_flags & FLAG_ALWAYS_USE_EKF;
    }

    NavEKF2 &EKF2;
    NavEKF3 &EKF3;
    bool _ekf2_started;
    bool _ekf3_started;
    bool _force_ekf;
    Matrix3f _dcm_matrix;
    Vector3f _dcm_attitude;
    Vector3f _gyro_drift;
    Vector3f _gyro_estimate;
    Vector3f _accel_ef_ekf[INS_MAX_INSTANCES];
    Vector3f _accel_ef_ekf_blended;
    const uint16_t startup_delay_ms = 1000;
    uint32_t start_time_ms = 0;
    Flags _ekf_flags;

    uint8_t ekf_type(void) const;
    void update_DCM(bool skip_ins_update);
    void update_EKF2(void);
    void update_EKF3(void);

    // get the index of the current primary IMU
    uint8_t get_primary_IMU_index(void) const;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL *_sitl;
    uint32_t _last_body_odm_update_ms = 0;
    void update_SITL(void);
#endif    
};
