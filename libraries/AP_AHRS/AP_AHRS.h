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
 *  AHRS (Attitude Heading Reference System) frontend interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_config.h"

#include <AP_HAL/Semaphores.h>

#include "AP_AHRS_Backend.h"
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

#include "AP_AHRS_DCM.h"
#include "AP_AHRS_SIM.h"
#include "AP_AHRS_External.h"

// forward declare view class
class AP_AHRS_View;

#define AP_AHRS_NAVEKF_SETTLE_TIME_MS 20000     // time in milliseconds the ekf needs to settle after being started


// fwd declare GSF estimator
class EKFGSF_yaw;

class AP_AHRS {
    friend class AP_AHRS_View;
public:

    // copy this into our namespace
    using Status = NavFilterStatusBit;

    enum Flags {
        FLAG_ALWAYS_USE_EKF = 0x1,
    };

    // has_status returns information about the EKF health and
    // capabilities.  It is currently invalid to call this when a
    // backend is in charge which returns false for get_filter_status
    // - so this will simply return false for DCM, for example.
    bool has_status(Status status) const;

    // Constructor
    AP_AHRS(uint8_t flags = 0);

    // initialise
    void init(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS);

    // get singleton instance
    static AP_AHRS *get_singleton() {
        return _singleton;
    }

    // periodically checks to see if we should update the AHRS
    // orientation (e.g. based on the AHRS_ORIENTATION parameter)
    // allow for runtime change of orientation
    // this makes initial config easier
    void update_orientation();

    // allow threads to lock against AHRS update
    HAL_Semaphore &get_semaphore(void) {
        return _rsem;
    }

    // return the smoothed gyro vector corrected for drift
    const Vector3f &get_gyro(void) const { return state.gyro_estimate; }

    // return the current drift correction integrator value
    const Vector3f &get_gyro_drift(void) const { return state.gyro_drift; }

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift();

    void            update(bool skip_ins_update=false);
    void            reset();

    // get current location estimate
    bool get_location(Location &loc) const;

    // get latest altitude estimate above ground level in meters and validity flag
    bool get_hagl(float &hagl) const WARN_IF_UNUSED;

    // status reporting of estimated error
    float           get_error_rp() const;
    float           get_error_yaw() const;

    /*
     * wind estimation support
     */

    // enable wind estimation
    void set_wind_estimation_enabled(bool b) { wind_estimation_enabled = b; }

    // wind_estimation_enabled returns true if wind estimation is enabled
    bool get_wind_estimation_enabled() const { return wind_estimation_enabled; }

    // return a wind estimation vector, in m/s; returns 0,0,0 on failure
    const Vector3f &wind_estimate() const { return state.wind_estimate; }

    // return a wind estimation vector, in m/s; returns 0,0,0 on failure
    bool wind_estimate(Vector3f &wind) const;

    // Determine how aligned heading_deg is with the wind. Return result
    // is 1.0 when perfectly aligned heading into wind, -1 when perfectly
    // aligned with-wind, and zero when perfect cross-wind. There is no
    // distinction between a left or right cross-wind. Wind speed is ignored
    float wind_alignment(const float heading_deg) const;

    // returns forward head-wind component in m/s. Negative means tail-wind
    float head_wind(void) const;

    // instruct DCM to update its wind estimate:
    void estimate_wind() {
#if AP_AHRS_DCM_ENABLED
        dcm.estimate_wind();
#endif
    }

#if AP_AHRS_EXTERNAL_WIND_ESTIMATE_ENABLED
    void set_external_wind_estimate(float speed, float direction) {
        dcm.set_external_wind_estimate(speed, direction);
    }
#endif

    // return the parameter AHRS_WIND_MAX in metres per second
    uint8_t get_max_wind() const {
        return _wind_max;
    }

    /*
     * airspeed support
     */

    // get apparent to true airspeed ratio
    float get_EAS2TAS(void) const;

    // get air density / sea level density - decreases as altitude climbs
    float get_air_density_ratio(void) const;
    
    // return an (equivalent) airspeed estimate if available. return
    // true if airspeed_ret is valid.  This value may be derived from
    // airspeed data or synthesised from other sources.
    bool airspeed_EAS(float &airspeed_ret) const;

    enum AirspeedEstimateType : uint8_t {
        NO_NEW_ESTIMATE = 0,
        AIRSPEED_SENSOR = 1,
        DCM_SYNTHETIC = 2,
        EKF3_SYNTHETIC = 3,
        SIM = 4,
    };

    // return an (equivalent) airspeed estimate if available. return
    // true if airspeed_ret is valid. This value may be derived from
    // airspeed data or synthesised from other sources (the type
    // return parameter allows you to distinguish).
    bool airspeed_EAS(float &airspeed_ret, AirspeedEstimateType &type) const;

    // return true if the current AHRS airspeed estimate (from airspeed_estimate method) is directly derived from an airspeed sensor
    bool using_airspeed_sensor() const;

    // return a true airspeed (navigation airspeed) if
    // available. return true if airspeed_ret is valid.  This value
    // may be derived from actual airspeed sensor data or synthesized
    // from other sources.
    bool airspeed_TAS(float &airspeed_ret) const;

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    bool airspeed_vector_TAS(Vector3f &vec) const;

    // return the innovation in m/s, innovation variance in (m/s)^2 and age in msec of the last TAS measurement processed
    // returns false if the data is unavailable
    bool airspeed_health_data(float &innovation, float &innovationVariance, uint32_t &age_ms) const;

    // return true if a airspeed sensor is enabled
    bool airspeed_sensor_enabled(void) const {
        // FIXME: make this a method on the active backend
        return AP_AHRS_Backend::airspeed_sensor_enabled();
    }

    // return true if a airspeed from a specific airspeed sensor is enabled
    bool airspeed_sensor_enabled(uint8_t airspeed_index) const {
        // FIXME: make this a method on the active backend
        return AP_AHRS_Backend::airspeed_sensor_enabled(airspeed_index);
    }

    // true if compass is being used
    bool use_compass();

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const WARN_IF_UNUSED;

    // return secondary attitude solution if available, as eulers in radians
    bool get_secondary_attitude(Vector3f &eulers) const {
        eulers = state.secondary_attitude;
        return state.secondary_attitude_ok;
    }

    // return secondary attitude solution if available, as quaternion
    bool get_secondary_quaternion(Quaternion &quat) const {
        quat = state.secondary_quat;
        return state.secondary_quat_ok;
    }

    // return secondary position solution if available
    bool get_secondary_position(Location &loc) const {
        loc = state.secondary_pos;
        return state.secondary_pos_ok;
    }

    // EKF has a better ground speed vector estimate
    const Vector2f &groundspeed_vector() const { return state.ground_speed_vec; }

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float groundspeed(void) const { return state.ground_speed; }

    const Vector3f &get_accel_ef() const {
        return state.accel_ef;
    }

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const {
        ret = state.corrected_dv;
        dt = state.corrected_dv_dt;
    }

    // set the EKF's origin location in 10e7 degrees.  This should only
    // be called when the EKF has no absolute position reference (i.e. GPS)
    // from which to decide the origin on its own
    bool set_origin(const Location &loc) WARN_IF_UNUSED;

#if AP_AHRS_POSITION_RESET_ENABLED
    // Set the EKF's NE horizontal position states and their corresponding variances from the supplied WGS-84 location
    // and 1-sigma horizontal position uncertainty. This can be used when the EKF is dead reckoning to periodically
    // correct the position. If the EKF is is still using data from a postion sensor such as GPS, the position set
    // will not be performed.
    // pos_accuracy is the standard deviation of the horizontal position uncertainty in metres.
    // The altitude element of the location is not used.
    // Returns true if the set was successful.
    bool handle_external_position_estimate(const Location &loc, float pos_accuracy, uint32_t timestamp_);
#endif

    // returns the inertial navigation origin in lat/lon/alt
    bool get_origin(Location &ret) const WARN_IF_UNUSED;

    bool have_inertial_nav() const;

    // return a ground velocity in meters/second, North/East/Down
    // order. Must only be called if have_inertial_nav() is true
    bool get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED;

    // return the relative position NED from either home or origin
    // return true if the estimate is valid
    bool get_relative_position_NED_home(Vector3f &vec) const WARN_IF_UNUSED;
    bool get_relative_position_NED_origin(Vector3p &vec) const WARN_IF_UNUSED;
    bool get_relative_position_NED_origin_float(Vector3f &vec) const WARN_IF_UNUSED;

    // return the relative position NE from home or origin
    // return true if the estimate is valid
    bool get_relative_position_NE_home(Vector2f &posNE) const WARN_IF_UNUSED;
    bool get_relative_position_NE_origin(Vector2p &posNE) const WARN_IF_UNUSED;
    bool get_relative_position_NE_origin_float(Vector2f &posNE) const WARN_IF_UNUSED;

    // return the relative position down from home or origin
    // baro will be used for the _home relative one if the EKF isn't
    void get_relative_position_D_home(float &posD) const;
    bool get_relative_position_D_origin(postype_t &posD) const WARN_IF_UNUSED;
    bool get_relative_position_D_origin_float(float &posD) const WARN_IF_UNUSED;

    // return location corresponding to vector relative to the
    // vehicle's origin
    bool get_location_from_origin_offset_NED(Location &loc, const Vector3p &offset_ned) const WARN_IF_UNUSED;
    bool get_location_from_home_offset_NED(Location &loc, const Vector3p &offset_ned) const WARN_IF_UNUSED;

    // get velocity down in m/s.  This returns get_velocity_NED.z() if available, otherwise falls back to get_vert_pos_rate_D()
    // if high_vibes is true then this is equivalent to get_vert_pos_rate_D
    bool get_velocity_D(float &velD, bool high_vibes = false) const WARN_IF_UNUSED;

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate_D(float &velocity) const;

    // write optical flow measurements to EKF
    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, const float heightOverride);

    // retrieve latest corrected optical flow samples (used for calibration)
    bool getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const;

    // write body odometry measurements to the EKF
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset);

    // Writes the default equivalent airspeed and its 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
    void writeDefaultAirSpeed(float airspeed, float uncertainty);

    // Write position and quaternion data from an external navigation system
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms);

    // Write velocity data from an external navigation system
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms);

    // Write terrain (derived from SRTM) altitude in meters above sea level
    void writeTerrainAMSL(float alt_amsl_m);

    // get speed limit
    void getControlLimits(float &ekfGndSpdLimit, float &controlScaleXY) const;
    float getControlScaleZ(void) const;

    // is the AHRS subsystem healthy?
    bool healthy() const;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const;

    // true if the AHRS has completed initialisation
    bool initialised() const;

#if AP_AHRS_DCM_ENABLED
    // return true if *DCM* yaw has been initialised
    bool dcm_yaw_initialised(void) const {
        return dcm.yaw_initialised();
    }
#endif

    // get_filter_status - returns filter status as a series of flags
    bool get_filter_status(nav_filter_status &status) const;

    // get compass offset estimates
    // true if offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng);

    // return the amount of NE position change in meters due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos);

    // return the amount of NE velocity change in meters/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // return the amount of vertical position change due to the last reset in meters
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosDownReset(float &posDelta);

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum();

    // send a EKF_STATUS_REPORT for current EKF
    void send_ekf_status_report(class GCS_MAVLINK &link) const;

    // get_hgt_ctrl_limit - get maximum height to be observed by the control loops in meters and a validity flag
    // this is used to limit height during optical flow navigation
    // it will return invalid when no limiting is required
    bool get_hgt_ctrl_limit(float &limit) const;

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // this is not related to terrain following
    void set_terrain_hgt_stable(bool stable);

    // return the innovations for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const;

    // returns true when the state estimates are significantly degraded by vibration
    bool is_vibration_affected() const;

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const;

    // get a source's velocity innovations
    // returns true on success and results are placed in innovations and variances arguments
    bool get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED;

    // returns the expected NED magnetic field
    bool get_mag_field_NED(Vector3f& ret) const;

    // returns the estimated magnetic field offsets in body frame
    bool get_mag_field_correction(Vector3f &ret) const;

    // return the index of the airspeed we should use for airspeed measurements
    // with multiple airspeed sensors and airspeed affinity in EKF3, it is possible to have switched
    // over to a lane not using the primary airspeed sensor, so AHRS should know which airspeed sensor
    // to use, i.e, the one being used by the primary lane. A lane switch could have happened due to an 
    // airspeed sensor fault, which makes this even more necessary
    uint8_t get_active_airspeed_index() const;

    // return the index of the primary core or -1 if no primary core selected
    int8_t get_primary_core_index() const { return state.primary_core; }

    // get the index of the current primary accelerometer sensor
    uint8_t get_primary_accel_index(void) const { return state.primary_accel; }

    // get the index of the current primary gyro sensor
    uint8_t get_primary_gyro_index(void) const { return state.primary_gyro; }

    // see if EKF lane switching is possible to avoid EKF failsafe
    void check_lane_switch(void);

    // request EKF yaw reset to try and avoid the need for an EKF lane switch or failsafe
    void request_yaw_reset(void);

    // set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
    void set_posvelyaw_source_set(AP_NavEKF_Source::SourceSetSelection source_set_idx);

    //returns index of active source set used, 0=primary, 1=secondary, 2=tertiary
    uint8_t get_posvelyaw_source_set() const;

    void Log_Write();

    // check if non-compass sensor is providing yaw.  Allows compass pre-arm checks to be bypassed
    bool using_noncompass_for_yaw(void) const;

    // check if external nav is providing yaw
    bool using_extnav_for_yaw(void) const;

    // set and save the ALT_M_NSE parameter value
    void set_alt_measurement_noise(float noise);

    enum class EKFType : uint8_t {
#if AP_AHRS_DCM_ENABLED
        DCM = 0,
#endif
#if HAL_NAVEKF3_AVAILABLE
        THREE = 3,
#endif
#if HAL_NAVEKF2_AVAILABLE
        TWO = 2,
#endif
#if AP_AHRS_SIM_ENABLED
        SIM = 10,
#endif
#if AP_AHRS_EXTERNAL_ENABLED
        EXTERNAL = 11,
#endif
    };

    // returns a canonicalised and valid EKFType, as opposed to the raw
    // parameter value which may be any value the user has set
    EKFType configured_ekf_type(void) const;

    // set the selected ekf type, for RC aux control
    void set_ekf_type(EKFType ahrs_type) {
        _ekf_type.set(ahrs_type);
    }
    
    // these are only out here so vehicles can reference them for parameters
#if HAL_NAVEKF2_AVAILABLE
    NavEKF2 EKF2;
#endif
#if HAL_NAVEKF3_AVAILABLE
    NavEKF3 EKF3;
#endif

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // create a view
    AP_AHRS_View *create_view(enum Rotation rotation, float pitch_trim_deg=0);

    // write AOA and SSA information to dataflash logs:
    void Write_AOA_SSA(void) const;

    // return AOA
    float getAOA(void) const { return _AOA; }

    // return SSA
    float getSSA(void) const { return _SSA; }

    /*
     * trim-related functions
     */

    // get trim
    const Vector3f &get_trim() const { return _trim.get(); }

    // set trim
    void set_trim(const Vector3f &new_trim);

    // add_trim - adjust the roll and pitch trim up to a total of 10 degrees
    void add_trim(float roll_in_radians, float pitch_in_radians, bool save_to_eeprom = true);

    // trim rotation matrices:
    const Matrix3f& get_rotation_autopilot_body_to_vehicle_body(void) const { return _rotation_autopilot_body_to_vehicle_body; }
    const Matrix3f& get_rotation_vehicle_body_to_autopilot_body(void) const { return _rotation_vehicle_body_to_autopilot_body; }

    // Logging functions
    void Log_Write_Home_And_Origin();
    void Write_Attitude(const Vector3f &targets) const;

    enum class LogOriginType {
        ekf_origin = 0,
        ahrs_home = 1
    };
    void Write_Origin(LogOriginType origin_type, const Location &loc) const; 
    void write_video_stabilisation() const;

    // return a smoothed and corrected gyro vector in radians/second
    // using the latest ins data (which may not have been consumed by
    // the EKF yet)
    Vector3f get_gyro_latest(void) const;

    // get yaw rate in earth frame in radians/sec
    float get_yaw_rate_earth(void) const {
        return get_gyro() * get_rotation_body_to_ned().c;
    }

    /*
     * home-related functionality
     */

    // get the home location. This is const to prevent any changes to
    // home without telling AHRS about the change
    const Location &get_home(void) const {
        return _home;
    }

    // functions to handle locking of home.  Some vehicles use this to
    // allow GCS to lock in a home location.
    void lock_home() {
        _home_locked = true;
    }
    bool home_is_locked() const {
        return _home_locked;
    }

    // returns true if home is set
    bool home_is_set(void) const {
        return _home_is_set;
    }

    // set the home location in 10e7 degrees. This should be called
    // when the vehicle is at this position. It is assumed that the
    // current barometer and GPS altitudes correspond to this altitude
    bool set_home(const Location &loc) WARN_IF_UNUSED;

    /*
     * Attitude-related public methods and attributes:
     */

#if AP_SCRIPTING_ENABLED
    // deprecated functions for accessing rpy.  Do not use, these will
    // be removed.
    float get_roll() const { return roll; }
    float get_pitch() const { return pitch; }
    float get_yaw() const { return yaw; }
#endif  // AP_SCRIPTING_ENABLED

    // roll/pitch/yaw euler angles, all in radians
    float get_roll_rad() const { return roll; }
    float get_pitch_rad() const { return pitch; }
    float get_yaw_rad() const { return yaw; }

    // roll/pitch/yaw euler angles, all in degrees
    float get_roll_deg() const { return rpy_deg[0]; }
    float get_pitch_deg() const { return rpy_deg[1]; }
    float get_yaw_deg() const { return rpy_deg[2]; }

    // helper trig value accessors
    float cos_roll() const  {
        return _cos_roll;
    }
    float cos_pitch() const {
        return _cos_pitch;
    }
    float cos_yaw() const   {
        return _cos_yaw;
    }
    float sin_roll() const  {
        return _sin_roll;
    }
    float sin_pitch() const {
        return _sin_pitch;
    }
    float sin_yaw() const   {
        return _sin_yaw;
    }

    // floating point Euler angles (Degrees)
    float rpy_deg[3];

    // integer Euler angles (Degrees * 100)
    int32_t roll_sensor;
    int32_t pitch_sensor;
    int32_t yaw_sensor;

    const Matrix3f &get_rotation_body_to_ned(void) const { return state.dcm_matrix; }

    // return a Quaternion representing our current attitude in NED frame
    void get_quat_body_to_ned(Quaternion &quat) const;

#if AP_AHRS_DCM_ENABLED
    // get rotation matrix specifically from DCM backend (used for
    // compass calibrator)
    const Matrix3f &get_DCM_rotation_body_to_ned(void) const {
        return dcm_estimates.dcm_matrix;
    }
#endif

    // rotate a 2D vector from earth frame to body frame
    // in result, x is forward, y is right
    Vector2f earth_to_body2D(const Vector2f &ef_vector) const;

    // rotate a 2D vector from earth frame to body frame
    // in input, x is forward, y is right
    Vector2f body_to_earth2D(const Vector2f &bf) const WARN_IF_UNUSED;
    Vector2p body_to_earth2D_p(const Vector2p &bf) const WARN_IF_UNUSED;

    // convert a vector from body to earth frame
    Vector3f body_to_earth(const Vector3f &v) const;

    // convert a vector from earth to body frame
    Vector3f earth_to_body(const Vector3f &v) const;

    /*
     * methods for the benefit of LUA bindings
     */
    // return current vibration vector for primary IMU
    Vector3f get_vibration(void) const;

    // return primary accels
    const Vector3f &get_accel(void) const {
        return AP::ins().get_accel(_get_primary_accel_index());
    }

    // return primary accel bias. This should be subtracted from
    // get_accel() vector to get best current body frame accel
    // estimate
    const Vector3f &get_accel_bias(void) const {
        return state.accel_bias;
    }
    
    /*
     * AHRS is used as a transport for vehicle-takeoff-expected and
     * vehicle-landing-expected:
     */
    void set_takeoff_expected(bool b);

    bool get_takeoff_expected(void) const {
        return takeoff_expected;
    }

    void set_touchdown_expected(bool b);

    bool get_touchdown_expected(void) const {
        return touchdown_expected;
    }

    /*
     * fly_forward is set by the vehicles to indicate the vehicle
     * should generally be moving in the direction of its heading.
     * It is an additional piece of information that the backends can
     * use to provide additional and/or improved estimates.
     */
    void set_fly_forward(bool b) {
        fly_forward = b;
    }
    bool get_fly_forward(void) const {
        return fly_forward;
    }

    /* we modify our behaviour based on what sort of vehicle the
     * vehicle code tells us we are.  This information is also pulled
     * from AP_AHRS by other libraries
     */
    enum class VehicleClass : uint8_t {
        UNKNOWN,
        GROUND,
        COPTER,
        FIXED_WING,
        SUBMARINE,
    };
    VehicleClass get_vehicle_class(void) const {
        return _vehicle_class;
    }
    void set_vehicle_class(VehicleClass vclass) {
        _vehicle_class = vclass;
    }

    // get the view
    AP_AHRS_View *get_view(void) const { return _view; };

    // get access to an EKFGSF_yaw estimator
    const EKFGSF_yaw *get_yaw_estimator(void) const;

private:

    // roll/pitch/yaw euler angles, all in radians
    float roll;
    float pitch;
    float yaw;

    // optional view class
    AP_AHRS_View *_view;

    static AP_AHRS *_singleton;

    /* we modify our behaviour based on what sort of vehicle the
     * vehicle code tells us we are.  This information is also pulled
     * from AP_AHRS by other libraries
     */
    VehicleClass _vehicle_class{VehicleClass::UNKNOWN};

    // multi-thread access support
    HAL_Semaphore _rsem;

    /*
     * Parameters
     */
    AP_Int8 _wind_max;
    AP_Int8 _board_orientation;
    AP_Enum<EKFType> _ekf_type;

    /*
     * DCM-backend parameters; it takes references to these
     */
    // settable parameters
    AP_Float _kp_yaw;
    AP_Float _kp;
    AP_Float gps_gain;

    AP_Float beta;

    AP_Enum<GPSUse> _gps_use;
    AP_Int8 _gps_minsats;

    EKFType active_EKF_type(void) const { return state.active_EKF; }

    bool always_use_EKF() const {
        return _ekf_flags & FLAG_ALWAYS_USE_EKF;
    }

    // check all cores providing consistent attitudes for prearm checks
    bool attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const;
    // convenience method for setting error string:
    void set_failure_inconsistent_message(const char *estimator, const char *axis, float diff_rad, char *failure_msg, const uint8_t failure_msg_len) const;

    /*
     * Attitude-related private methods and attributes:
     */
    // calculate sin/cos of roll/pitch/yaw from rotation
    void calc_trig(const Matrix3f &rot,
                   float &cr, float &cp, float &cy,
                   float &sr, float &sp, float &sy) const;

    // update_trig - recalculates _cos_roll, _cos_pitch, etc based on latest attitude
    //      should be called after _dcm_matrix is updated
    void update_trig(void);

    // update roll_sensor, pitch_sensor and yaw_sensor
    void update_cd_values(void);

    // helper trig variables
    float _cos_roll{1.0f};
    float _cos_pitch{1.0f};
    float _cos_yaw{1.0f};
    float _sin_roll;
    float _sin_pitch;
    float _sin_yaw;

#if HAL_NAVEKF2_AVAILABLE
    void update_EKF2(void);
    bool _ekf2_started;
#endif
#if HAL_NAVEKF3_AVAILABLE
    bool _ekf3_started;
    void update_EKF3(void);
#endif

    const uint16_t startup_delay_ms = 1000;
    uint32_t start_time_ms;
    uint8_t _ekf_flags; // bitmask from Flags enumeration

    void update_DCM();

    /*
     * home-related state
     */
    void load_watchdog_home();
    bool _checked_watchdog_home;
    Location _home;
    bool _home_is_set :1;
    bool _home_locked :1;

    // avoid setting current state repeatedly across all cores on all EKFs:
    enum class TriState {
        False = 0,
        True = 1,
        UNKNOWN = 3,
    };

    TriState terrainHgtStableState = TriState::UNKNOWN;

    /*
     * private AOA and SSA-related state and methods
     */
    float _AOA, _SSA;
    uint32_t _last_AOA_update_ms;
    void update_AOA_SSA(void);

    EKFType last_active_ekf_type;

#if AP_AHRS_SIM_ENABLED
    void update_SITL(void);
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    void update_external(void);
#endif    

    /*
     * trim-related state and private methods:
     */

    // a vector to capture the difference between the controller and body frames
    AP_Vector3f         _trim;

    // cached trim rotations
    Vector3f _last_trim;

    Matrix3f _rotation_autopilot_body_to_vehicle_body;
    Matrix3f _rotation_vehicle_body_to_autopilot_body;

    // last time orientation was updated from AHRS_ORIENTATION:
    uint32_t last_orientation_update_ms;

    // updates matrices responsible for rotating vectors from vehicle body
    // frame to autopilot body frame from _trim variables
    void update_trim_rotation_matrices();

    /*
     * AHRS is used as a transport for vehicle-takeoff-expected and
     * vehicle-landing-expected:
     */
    // update takeoff/touchdown flags
    void update_flags();
    bool takeoff_expected;    // true if the vehicle is in a state that takeoff might be expected.  Ground effect may be in play.
    uint32_t takeoff_expected_start_ms;
    bool touchdown_expected;    // true if the vehicle is in a state that touchdown might be expected.  Ground effect may be in play.
    uint32_t touchdown_expected_start_ms;

    /*
     * wind estimation support
     */
    bool wind_estimation_enabled;

    /*
     * fly_forward is set by the vehicles to indicate the vehicle
     * should generally be moving in the direction of its heading.
     * It is an additional piece of information that the backends can
     * use to provide additional and/or improved estimates.
     */
    bool fly_forward; // true if we can assume the vehicle will be flying forward on its X axis

    // poke AP_Notify based on values from status
    void update_notify_from_filter_status(const nav_filter_status &status);

    /*
     * copy results from a backend over AP_AHRS canonical results.
     * This updates member variables like roll and pitch, as well as
     * updating derived values like sin_roll and sin_pitch.
     */
    void copy_estimates_from_backend_estimates(const AP_AHRS_Backend::Estimates &results);

    // write out secondary estimates:
    void Write_AHRS2(void) const;
    // write POS (canonical vehicle position) message out:
    void Write_POS(void) const;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool _airspeed_EAS(float &airspeed_ret, AirspeedEstimateType &status) const;

    // return secondary attitude solution if available, as eulers in radians
    bool _get_secondary_attitude(Vector3f &eulers) const;

    // return secondary attitude solution if available, as quaternion
    bool _get_secondary_quaternion(Quaternion &quat) const;

    // get ground speed 2D
    Vector2f _groundspeed_vector(void);

    // get active EKF type
    EKFType _active_EKF_type(void) const;

    // return a wind estimation vector, in m/s
    bool _wind_estimate(Vector3f &wind) const WARN_IF_UNUSED;

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool _airspeed_TAS(float &airspeed_ret) const;

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    bool _airspeed_TAS(Vector3f &vec) const;

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool _get_quaternion(Quaternion &quat) const WARN_IF_UNUSED;

    // return secondary position solution if available
    bool _get_secondary_position(Location &loc) const;

    // return ground speed estimate in meters/second. Used by ground vehicles.
    float _groundspeed(void);

    // Retrieves the corrected NED delta velocity in use by the inertial navigation
    void _getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const;

    // returns the inertial navigation origin in lat/lon/alt
    bool _get_origin(Location &ret) const WARN_IF_UNUSED;

    // return origin for a specified EKF type
    bool _get_origin(EKFType type, Location &ret) const;

    // return a ground velocity in meters/second, North/East/Down
    // order. Must only be called if have_inertial_nav() is true
    bool _get_velocity_NED(Vector3f &vec) const WARN_IF_UNUSED;

    // get secondary EKF type.  returns false if no secondary (i.e. only using DCM)
    bool _get_secondary_EKF_type(EKFType &secondary_ekf_type) const;

    // return the index of the primary core or -1 if no primary core selected
    int8_t _get_primary_core_index() const;

    // get the index of the current primary accelerometer sensor
    uint8_t _get_primary_accel_index(void) const;

    // get the index of the current primary gyro sensor
    uint8_t _get_primary_gyro_index(void) const;

    // get the index of the current primary IMU
    uint8_t _get_primary_IMU_index(void) const;

    // get current location estimate
    bool _get_location(Location &loc) const;

    // return true if a airspeed sensor should be used for the AHRS airspeed estimate
    bool _should_use_airspeed_sensor(uint8_t airspeed_index) const;
    
    /*
      update state structure
     */
    void update_state(void);

    // returns an EKF type to be used as active if we decide the
    // primary is not good enough.
    EKFType fallback_active_EKF_type(void) const;

    /*
      state updated at the end of each update() call
     */
    struct {
        EKFType active_EKF;
        uint8_t primary_IMU;
        uint8_t primary_gyro;
        uint8_t primary_accel;
        uint8_t primary_core;
        Vector3f gyro_estimate;
        Matrix3f dcm_matrix;
        Vector3f gyro_drift;
        Vector3f accel_ef;
        Vector3f accel_bias;
        Vector3f wind_estimate;
        bool wind_estimate_ok;
        float EAS2TAS;
        bool airspeed_EAS_ok;
        float airspeed_EAS;
        AirspeedEstimateType airspeed_estimate_type;
        bool airspeed_TAS_ok;
        float airspeed_TAS;
        Vector3f airspeed_TAS_vec;
        bool airspeed_TAS_vec_ok;
        Quaternion quat;
        bool quat_ok;
        Vector3f secondary_attitude;
        bool secondary_attitude_ok;
        Quaternion secondary_quat;
        bool secondary_quat_ok;
        Location location;
        bool location_ok;
        Location secondary_pos;
        bool secondary_pos_ok;
        Vector2f ground_speed_vec;
        float ground_speed;
        Vector3f corrected_dv;
        float corrected_dv_dt;
        Location origin;
        bool origin_ok;
        Vector3f velocity_NED;
        bool velocity_NED_ok;
    } state;

    /*
     *  backends (and their results)
     */
#if AP_AHRS_DCM_ENABLED
    AP_AHRS_DCM dcm{_kp_yaw, _kp, gps_gain, beta, _gps_use, _gps_minsats};
    struct AP_AHRS_Backend::Estimates dcm_estimates;
#endif
#if AP_AHRS_SIM_ENABLED
#if HAL_NAVEKF3_AVAILABLE
    AP_AHRS_SIM sim{EKF3};
#else
    AP_AHRS_SIM sim;
#endif
    struct AP_AHRS_Backend::Estimates sim_estimates;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    AP_AHRS_External external;
    struct AP_AHRS_Backend::Estimates external_estimates;
#endif

    enum class Options : uint16_t {
        DISABLE_DCM_FALLBACK_FW=(1U<<0),
        DISABLE_DCM_FALLBACK_VTOL=(1U<<1),
        DISABLE_AIRSPEED_EKF_CHECK=(1U<<2),
    };
    AP_Int16 _options;
    
    bool option_set(Options option) const {
        return (_options & uint16_t(option)) != 0;
    }

    // true when we have completed the common origin setup
    bool done_common_origin;
};

namespace AP {
    AP_AHRS &ahrs();
};
