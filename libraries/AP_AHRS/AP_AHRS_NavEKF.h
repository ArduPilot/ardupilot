/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_AHRS_NAVEKF_H__
#define __AP_AHRS_NAVEKF_H__
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
#include "AP_AHRS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

#define AP_AHRS_NAVEKF_AVAILABLE 1
#define AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000     // time in milliseconds the ekf needs to settle after being started

class AP_AHRS_NavEKF : public AP_AHRS_DCM
{
public:
    enum Flags {
        FLAG_NONE = 0,
        FLAG_ALWAYS_USE_EKF = 0x1,
    };

    // Constructor
    AP_AHRS_NavEKF(AP_InertialSensor &ins, AP_Baro &baro, AP_GPS &gps, RangeFinder &rng,
                   NavEKF &_EKF1, NavEKF2 &_EKF2, Flags flags = FLAG_NONE);

    // return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const;
    const Matrix3f &get_rotation_body_to_ned(void) const;

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const;

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift(void);

    void            update(void);
    void            reset(bool recover_eulers = false);

    // reset the current attitude, used on new IMU calibration
    void reset_attitude(const float &roll, const float &pitch, const float &yaw);

    // dead-reckoning support
    bool get_position(struct Location &loc) const;

    // get latest altitude estimate above ground level in metres and validity flag
    bool get_hagl(float &hagl) const;

    // status reporting of estimated error
    float           get_error_rp(void) const;
    float           get_error_yaw(void) const;

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate(void);

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(float *airspeed_ret) const;

    // true if compass is being used
    bool use_compass(void);

    // we will need to remove these to fully hide which EKF we are using
    NavEKF &get_NavEKF(void) {
        return EKF1;
    }
    const NavEKF &get_NavEKF_const(void) const {
        return EKF1;
    }

    NavEKF2 &get_NavEKF2(void) {
        return EKF2;
    }
    const NavEKF2 &get_NavEKF2_const(void) const {
        return EKF2;
    }

    // return secondary attitude solution if available, as eulers in radians
    bool get_secondary_attitude(Vector3f &eulers);

    // return secondary position solution if available
    bool get_secondary_position(struct Location &loc);

    // EKF has a better ground speed vector estimate
    Vector2f groundspeed_vector(void);

    const Vector3f &get_accel_ef(uint8_t i) const;
    const Vector3f &get_accel_ef() const {
        return get_accel_ef(_ins.get_primary_accel());
    };

    // blended accelerometer values in the earth frame in m/s/s
    const Vector3f &get_accel_ef_blended(void) const;

    // set home location
    void set_home(const Location &loc);

    // returns the inertial navigation origin in lat/lon/alt
    bool get_origin(Location &ret) const;

    bool have_inertial_nav(void) const;

    bool get_velocity_NED(Vector3f &vec) const;
    bool get_relative_position_NED(Vector3f &vec) const;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity);

    // write optical flow measurements to EKF
    void writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas);

    // inibit GPS useage
    uint8_t setInhibitGPS(void);

    // get speed limit
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler);

    void set_ekf_use(bool setting);

    // is the AHRS subsystem healthy?
    bool healthy(void) const;

    // true if the AHRS has completed initialisation
    bool initialised(void) const;

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const;

    // get compass offset estimates
    // true if offsets are valid
    bool getMagOffsets(Vector3f &magOffsets);

    // report any reason for why the backend is refusing to initialise
    const char *prearm_failure_reason(void) const override;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng) const;

    // return the amount of NE position change in metres due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const;

    // return the amount of NE velocity change in metres/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum(void);

    // send a EKF_STATUS_REPORT for current EKF
    void send_ekf_status_report(mavlink_channel_t chan);
    
    // get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
    // this is used to limit height during optical flow navigation
    // it will return invalid when no limiting is required
    bool get_hgt_ctrl_limit(float &limit) const;

    // get_llh - updates the provided location with the latest calculated location including absolute altitude
    //  returns true on success (i.e. the EKF knows it's latest position), false on failure
    bool get_location(struct Location &loc) const;

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates prefect consistency between the measurement and the EKF solution and a value of of 1 is the maximum
    // inconsistency that will be accpeted by the filter
    // boolean false is returned if variances are not available
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    // returns the expected NED magnetic field
    bool get_mag_field_NED(Vector3f& ret) const;

    // returns the estimated magnetic field offsets in body frame
    bool get_mag_field_correction(Vector3f &ret) const;

    void setTakeoffExpected(bool val);
    void setTouchdownExpected(bool val);

private:
    enum EKF_TYPE {EKF_TYPE_NONE=0,
                   EKF_TYPE1=1,
                   EKF_TYPE2=2
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                   ,EKF_TYPE_SITL=10
#endif
    };
    EKF_TYPE active_EKF_type(void) const;

    bool always_use_EKF() const {
        return _flags & FLAG_ALWAYS_USE_EKF;
    }

    NavEKF &EKF1;
    NavEKF2 &EKF2;
    bool ekf1_started = false;
    bool ekf2_started = false;
    Matrix3f _dcm_matrix;
    Vector3f _dcm_attitude;
    Vector3f _gyro_bias;
    Vector3f _gyro_estimate;
    Vector3f _accel_ef_ekf[INS_MAX_INSTANCES];
    Vector3f _accel_ef_ekf_blended;
    const uint16_t startup_delay_ms = 1000;
    uint32_t start_time_ms = 0;
    Flags _flags;

    uint8_t ekf_type(void) const;
    void update_DCM(void);
    void update_EKF1(void);
    void update_EKF2(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL *_sitl;
    void update_SITL(void);
#endif    
};
#endif

#endif // __AP_AHRS_NAVEKF_H__
