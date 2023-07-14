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

#include "AP_AHRS_config.h"

#if AP_AHRS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_AHRS.h"
#include "AP_AHRS_View.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Module/AP_Module.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_CustomRotations/AP_CustomRotations.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#define ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD radians(10)
#define ATTITUDE_CHECK_THRESH_YAW_RAD radians(20)

#ifndef HAL_AHRS_EKF_TYPE_DEFAULT
#define HAL_AHRS_EKF_TYPE_DEFAULT 3
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS::var_info[] = {
    // index 0 and 1 are for old parameters that are no longer not used

    // @Param: GPS_GAIN
    // @DisplayName: AHRS GPS gain
    // @Description: This controls how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
    // @Range: 0.0 1.0
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("GPS_GAIN",  2, AP_AHRS, gps_gain, 1.0f),

    // @Param: GPS_USE
    // @DisplayName: AHRS use GPS for DCM navigation and position-down
    // @Description: This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight. Currently this affects only the DCM-based AHRS: the EKF uses GPS according to its own parameters. A value of 2 means to use GPS for height as well as position - both in DCM estimation and when determining altitude-above-home.
    // @Values: 0:Disabled,1:Use GPS for DCM position,2:Use GPS for DCM position and height
    // @User: Advanced
    AP_GROUPINFO("GPS_USE",  3, AP_AHRS, _gps_use, float(GPSUse::Enable)),

    // @Param: YAW_P
    // @DisplayName: Yaw P
    // @Description: This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
    // @Range: 0.1 0.4
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("YAW_P", 4,    AP_AHRS, _kp_yaw, 0.2f),

    // @Param: RP_P
    // @DisplayName: AHRS RP_P
    // @Description: This controls how fast the accelerometers correct the attitude
    // @Range: 0.1 0.4
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("RP_P",  5,    AP_AHRS, _kp, 0.2f),

    // @Param: WIND_MAX
    // @DisplayName: Maximum wind
    // @Description: This sets the maximum allowable difference between ground speed and airspeed. A value of zero means to use the airspeed as is. This allows the plane to cope with a failing airspeed sensor by clipping it to groundspeed plus/minus this limit. See ARSPD_OPTIONS and ARSPD_WIND_MAX to disable airspeed sensors.
    // @Range: 0 127
    // @Units: m/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("WIND_MAX",  6,    AP_AHRS, _wind_max, 0.0f),

    // NOTE: 7 was BARO_USE

    // @Param: TRIM_X
    // @DisplayName: AHRS Trim Roll
    // @Description: Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
    // @Units: rad
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Standard

    // @Param: TRIM_Y
    // @DisplayName: AHRS Trim Pitch
    // @Description: Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
    // @Units: rad
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Standard

    // @Param: TRIM_Z
    // @DisplayName: AHRS Trim Yaw
    // @Description: Not Used
    // @Units: rad
    // @Range: -0.1745 +0.1745
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TRIM", 8, AP_AHRS, _trim, 0),

    // @Param: ORIENTATION
    // @DisplayName: Board Orientation
    // @Description: Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. The label for each option is specified in the order of rotations for that orientation. This option takes affect on next boot. After changing you will need to re-level your vehicle. Firmware versions 4.2 and prior can use a CUSTOM (100) rotation to set the AHRS_CUSTOM_ROLL/PIT/YAW angles for AHRS orientation. Later versions provide two general custom rotations which can be used, Custom 1 and Custom 2, with CUST_ROT1_ROLL/PIT/YAW or CUST_ROT2_ROLL/PIT/YAW angles.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom 4.1 and older,101:Custom 1,102:Custom 2
    // @User: Advanced
    AP_GROUPINFO("ORIENTATION", 9, AP_AHRS, _board_orientation, 0),

    // @Param: COMP_BETA
    // @DisplayName: AHRS Velocity Complementary Filter Beta Coefficient
    // @Description: This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
    // @Range: 0.001 0.5
    // @Increment: .01
    // @User: Advanced
    AP_GROUPINFO("COMP_BETA",  10, AP_AHRS, beta, 0.1f),

    // @Param: GPS_MINSATS
    // @DisplayName: AHRS GPS Minimum satellites
    // @Description: Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_MINSATS", 11, AP_AHRS, _gps_minsats, 6),

    // NOTE: index 12 was for GPS_DELAY, but now removed, fixed delay
    // of 1 was found to be the best choice

    // 13 was the old EKF_USE

    // @Param: EKF_TYPE
    // @DisplayName: Use NavEKF Kalman filter for attitude and position estimation
    // @Description: This controls which NavEKF Kalman filter version is used for attitude and position estimation
    // @Values: 0:Disabled,2:Enable EKF2,3:Enable EKF3,11:ExternalAHRS
    // @User: Advanced
    AP_GROUPINFO("EKF_TYPE",  14, AP_AHRS, _ekf_type, HAL_AHRS_EKF_TYPE_DEFAULT),

    // @Param: CUSTOM_ROLL
    // @DisplayName: Board orientation roll offset
    // @Description: Autopilot mounting position roll offset. Positive values = roll right, negative values = roll left. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @User: Advanced

    // index 15

    // @Param: CUSTOM_PIT
    // @DisplayName: Board orientation pitch offset
    // @Description: Autopilot mounting position pitch offset. Positive values = pitch up, negative values = pitch down. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @User: Advanced

    // index 16

    // @Param: CUSTOM_YAW
    // @DisplayName: Board orientation yaw offset
    // @Description: Autopilot mounting position yaw offset. Positive values = yaw right, negative values = yaw left. This parameter is only used when AHRS_ORIENTATION is set to CUSTOM.
    // @Range: -180 180
    // @Units: deg
    // @Increment: 1
    // @User: Advanced

    // index 17

    // @Param: OPTIONS
    // @DisplayName: Optional AHRS behaviour
    // @Description: This controls optional AHRS behaviour. Setting DisableDCMFallbackFW will change the AHRS behaviour for fixed wing aircraft in fly-forward flight to not fall back to DCM when the EKF stops navigating. Setting DisableDCMFallbackVTOL will change the AHRS behaviour for fixed wing aircraft in non fly-forward (VTOL) flight to not fall back to DCM when the EKF stops navigating. 
    // @Bitmask: 0:DisableDCMFallbackFW, 1:DisableDCMFallbackVTOL
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  18, AP_AHRS, _options, 0),
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// constructor
AP_AHRS::AP_AHRS(uint8_t flags) :
    _ekf_flags(flags)
{
    _singleton = this;

    // load default values from var_info table
    AP_Param::setup_object_defaults(this, var_info);

#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduSub)
    // Copter and Sub force the use of EKF
    _ekf_flags |= AP_AHRS::FLAG_ALWAYS_USE_EKF;
#endif
    state.dcm_matrix.identity();

    // initialise the controller-to-autopilot-body trim state:
    _last_trim = _trim.get();
    _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, _last_trim.z);
    _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
}

// init sets up INS board orientation
void AP_AHRS::init()
{
    // EKF1 is no longer supported - handle case where it is selected
    if (_ekf_type.get() == 1) {
        AP_BoardConfig::config_error("EKF1 not available");
    }
#if !HAL_NAVEKF2_AVAILABLE && HAL_NAVEKF3_AVAILABLE
    if (_ekf_type.get() == 2) {
        _ekf_type.set(EKFType::THREE);
        EKF3.set_enable(true);
    }
#elif !HAL_NAVEKF3_AVAILABLE && HAL_NAVEKF2_AVAILABLE
    if (_ekf_type.get() == 3) {
        _ekf_type.set(EKFType::TWO);
        EKF2.set_enable(true);
    }
#endif

#if HAL_NAVEKF2_AVAILABLE && HAL_NAVEKF3_AVAILABLE
    // a special case to catch users who had AHRS_EKF_TYPE=2 saved and
    // updated to a version where EK2_ENABLE=0
    if (_ekf_type.get() == 2 && !EKF2.get_enable() && EKF3.get_enable()) {
        _ekf_type.set(EKFType::THREE);
    }
#endif

    last_active_ekf_type = (EKFType)_ekf_type.get();

    // init backends
#if AP_AHRS_DCM_ENABLED
    dcm.init();
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    external.init();
#endif

#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
    // convert to new custom rotation
    // PARAMETER_CONVERSION - Added: Nov-2021
    if (_board_orientation == ROTATION_CUSTOM_OLD) {
        _board_orientation.set_and_save(ROTATION_CUSTOM_1);
        AP_Param::ConversionInfo info;
        if (AP_Param::find_top_level_key_by_pointer(this, info.old_key)) {
            info.type = AP_PARAM_FLOAT;
            float rpy[3] = {};
            AP_Float rpy_param;
            for (info.old_group_element=15; info.old_group_element<=17; info.old_group_element++) {
                if (AP_Param::find_old_parameter(&info, &rpy_param)) {
                    rpy[info.old_group_element-15] = rpy_param.get();
                }
            }
            AP::custom_rotations().convert(ROTATION_CUSTOM_1, rpy[0], rpy[1], rpy[2]);
        }
    }
#endif // !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
}

// updates matrices responsible for rotating vectors from vehicle body
// frame to autopilot body frame from _trim variables
void AP_AHRS::update_trim_rotation_matrices()
{
    if (_last_trim == _trim.get()) {
        // nothing to do
        return;
    }

    _last_trim = _trim.get();
    _rotation_autopilot_body_to_vehicle_body.from_euler(_last_trim.x, _last_trim.y, _last_trim.z);
    _rotation_vehicle_body_to_autopilot_body = _rotation_autopilot_body_to_vehicle_body.transposed();
}

// return a Quaternion representing our current attitude in NED frame
void AP_AHRS::get_quat_body_to_ned(Quaternion &quat) const
{
    quat.from_rotation_matrix(get_rotation_body_to_ned());
}

// convert a vector from body to earth frame
Vector3f AP_AHRS::body_to_earth(const Vector3f &v) const
{
    return get_rotation_body_to_ned() * v;
}

// convert a vector from earth to body frame
Vector3f AP_AHRS::earth_to_body(const Vector3f &v) const
{
    return get_rotation_body_to_ned().mul_transpose(v);
}


// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS::reset_gyro_drift(void)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    // update DCM
#if AP_AHRS_DCM_ENABLED
    dcm.reset_gyro_drift();
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    external.reset_gyro_drift();
#endif

    // reset the EKF gyro bias states
#if HAL_NAVEKF2_AVAILABLE
    EKF2.resetGyroBias();
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.resetGyroBias();
#endif
}

/*
  update state structure after each update()
 */
void AP_AHRS::update_state(void)
{
    state.primary_IMU = _get_primary_IMU_index();
    state.primary_gyro = _get_primary_gyro_index();
    state.primary_accel = _get_primary_accel_index();
    state.primary_core = _get_primary_core_index();
    state.wind_estimate_ok = _wind_estimate(state.wind_estimate);
    state.EAS2TAS = AP_AHRS_Backend::get_EAS2TAS();
    state.airspeed_ok = _airspeed_estimate(state.airspeed, state.airspeed_estimate_type);
    state.airspeed_true_ok = _airspeed_estimate_true(state.airspeed_true);
    state.airspeed_vec_ok = _airspeed_vector_true(state.airspeed_vec);
    state.quat_ok = _get_quaternion(state.quat);
    state.secondary_attitude_ok = _get_secondary_attitude(state.secondary_attitude);
    state.secondary_quat_ok = _get_secondary_quaternion(state.secondary_quat);
    state.location_ok = _get_location(state.location);
    state.secondary_pos_ok = _get_secondary_position(state.secondary_pos);
    state.ground_speed_vec = _groundspeed_vector();
    state.ground_speed = _groundspeed();
    _getCorrectedDeltaVelocityNED(state.corrected_dv, state.corrected_dv_dt);
    state.origin_ok = _get_origin(state.origin);
    state.velocity_NED_ok = _get_velocity_NED(state.velocity_NED);
}

void AP_AHRS::update(bool skip_ins_update)
{
    // periodically checks to see if we should update the AHRS
    // orientation (e.g. based on the AHRS_ORIENTATION parameter)
    // allow for runtime change of orientation
    // this makes initial config easier
    update_orientation();

    if (!skip_ins_update) {
        // tell the IMU to grab some data
        AP::ins().update();
    }

    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);

    // see if we have to restore home after a watchdog reset:
    if (!_checked_watchdog_home) {
        load_watchdog_home();
        _checked_watchdog_home = true;
    }

    // drop back to normal priority if we were boosted by the INS
    // calling delay_microseconds_boost()
    hal.scheduler->boost_end();

    // update autopilot-body-to-vehicle-body from _trim parameters:
    update_trim_rotation_matrices();

#if AP_AHRS_DCM_ENABLED
    update_DCM();
#endif

    // update takeoff/touchdown flags
    update_flags();

#if AP_AHRS_SIM_ENABLED
    update_SITL();
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    update_external();
#endif
    
    if (_ekf_type == 2) {
        // if EK2 is primary then run EKF2 first to give it CPU
        // priority
#if HAL_NAVEKF2_AVAILABLE
        update_EKF2();
#endif
#if HAL_NAVEKF3_AVAILABLE
        update_EKF3();
#endif
    } else {
        // otherwise run EKF3 first
#if HAL_NAVEKF3_AVAILABLE
        update_EKF3();
#endif
#if HAL_NAVEKF2_AVAILABLE
        update_EKF2();
#endif
    }

#if AP_MODULE_SUPPORTED
    // call AHRS_update hook if any
    AP_Module::call_hook_AHRS_update(*this);
#endif

    // push gyros if optical flow present
    if (hal.opticalflow) {
        const Vector3f &exported_gyro_bias = get_gyro_drift();
        hal.opticalflow->push_gyro_bias(exported_gyro_bias.x, exported_gyro_bias.y);
    }

    if (_view != nullptr) {
        // update optional alternative attitude view
        _view->update();
    }

    // update AOA and SSA
    update_AOA_SSA();

#if HAL_GCS_ENABLED
    state.active_EKF = _active_EKF_type();
    if (state.active_EKF != last_active_ekf_type) {
        last_active_ekf_type = state.active_EKF;
        const char *shortname = "???";
        switch ((EKFType)state.active_EKF) {
#if AP_AHRS_DCM_ENABLED
        case EKFType::DCM:
            shortname = "DCM";
            break;
#endif
#if AP_AHRS_SIM_ENABLED
        case EKFType::SIM:
            shortname = "SIM";
            break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
        case EKFType::EXTERNAL:
            shortname = "External";
            break;
#endif
#if HAL_NAVEKF3_AVAILABLE
        case EKFType::THREE:
            shortname = "EKF3";
            break;
#endif
#if HAL_NAVEKF2_AVAILABLE
        case EKFType::TWO:
            shortname = "EKF2";
            break;
#endif
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS: %s active", shortname);
    }
#endif // HAL_GCS_ENABLED

    // update published state
    update_state();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    /*
      add timing jitter to simulate slow EKF response
     */
    const auto *sitl = AP::sitl();
    if (sitl->loop_time_jitter_us > 0) {
        hal.scheduler->delay_microseconds(random() % sitl->loop_time_jitter_us);
    }
#endif
}

/*
 * copy results from a backend over AP_AHRS canonical results.
 * This updates member variables like roll and pitch, as well as
 * updating derived values like sin_roll and sin_pitch.
 */
void AP_AHRS::copy_estimates_from_backend_estimates(const AP_AHRS_Backend::Estimates &results)
{
    roll = results.roll_rad;
    pitch = results.pitch_rad;
    yaw = results.yaw_rad;

    state.dcm_matrix = results.dcm_matrix;

    state.gyro_estimate = results.gyro_estimate;
    state.gyro_drift = results.gyro_drift;

    state.accel_ef = results.accel_ef;
    state.accel_bias = results.accel_bias;

    update_cd_values();
    update_trig();
}

#if AP_AHRS_DCM_ENABLED
void AP_AHRS::update_DCM()
{
    dcm.update();
    dcm.get_results(dcm_estimates);

    // we always update the vehicle's canonical roll/pitch/yaw from
    // DCM.  In normal operation this will usually be over-written by
    // an EKF or external AHRS.  This is long-held behaviour, but this
    // really shouldn't be doing this.

    // if (active_EKF_type() == EKFType::DCM) {
        copy_estimates_from_backend_estimates(dcm_estimates);
    // }
}
#endif

#if AP_AHRS_SIM_ENABLED
void AP_AHRS::update_SITL(void)
{
    sim.update();
    sim.get_results(sim_estimates);

    if (_active_EKF_type() == EKFType::SIM) {
        copy_estimates_from_backend_estimates(sim_estimates);
    }
}
#endif

void AP_AHRS::update_notify_from_filter_status(const nav_filter_status &status)
{
    AP_Notify::flags.gps_fusion = status.flags.using_gps; // Drives AP_Notify flag for usable GPS.
    AP_Notify::flags.gps_glitching = status.flags.gps_glitching;
    AP_Notify::flags.have_pos_abs = status.flags.horiz_pos_abs;
}

#if HAL_NAVEKF2_AVAILABLE
void AP_AHRS::update_EKF2(void)
{
    if (!_ekf2_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
#if HAL_LOGGING_ENABLED
        // if we're doing Replay logging then don't allow any data
        // into the EKF yet.  Don't allow it to block us for long.
        if (!hal.util->was_watchdog_reset()) {
            if (AP_HAL::millis() - start_time_ms < 5000) {
                if (!AP::logger().allow_start_ekf()) {
                    return;
                }
            }
        }
#endif

        if (AP_HAL::millis() - start_time_ms > startup_delay_ms) {
            _ekf2_started = EKF2.InitialiseFilter();
        }
    }
    if (_ekf2_started) {
        EKF2.UpdateFilter();
        if (_active_EKF_type() == EKFType::TWO) {
            Vector3f eulers;
            EKF2.getRotationBodyToNED(state.dcm_matrix);
            EKF2.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // Use the primary EKF to select the primary gyro
            const AP_InertialSensor &_ins = AP::ins();
            const int8_t primary_imu = EKF2.getPrimaryCoreIMUIndex();
            const uint8_t primary_gyro = primary_imu>=0?primary_imu:_ins.get_primary_gyro();
            const uint8_t primary_accel = primary_imu>=0?primary_imu:_ins.get_primary_accel();

            // get gyro bias for primary EKF and change sign to give gyro drift
            // Note sign convention used by EKF is bias = measurement - truth
            Vector3f drift;
            EKF2.getGyroBias(drift);
            state.gyro_drift = -drift;

            // use the same IMU as the primary EKF and correct for gyro drift
            state.gyro_estimate = _ins.get_gyro(primary_gyro) + state.gyro_drift;

            // get z accel bias estimate from active EKF (this is usually for the primary IMU)
            float &abias = state.accel_bias.z;
            EKF2.getAccelZBias(abias);

            // This EKF is currently using primary_imu, and a bias applies to only that IMU
            Vector3f accel = _ins.get_accel(primary_accel);
            accel.z -= abias;
            state.accel_ef = state.dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * accel;

            nav_filter_status filt_state;
            EKF2.getFilterStatus(filt_state);
            update_notify_from_filter_status(filt_state);
        }
    }
}
#endif

#if HAL_NAVEKF3_AVAILABLE
void AP_AHRS::update_EKF3(void)
{
    if (!_ekf3_started) {
        // wait 1 second for DCM to output a valid tilt error estimate
        if (start_time_ms == 0) {
            start_time_ms = AP_HAL::millis();
        }
#if HAL_LOGGING_ENABLED
        // if we're doing Replay logging then don't allow any data
        // into the EKF yet.  Don't allow it to block us for long.
        if (!hal.util->was_watchdog_reset()) {
            if (AP_HAL::millis() - start_time_ms < 5000) {
                if (!AP::logger().allow_start_ekf()) {
                    return;
                }
            }
        }
#endif
        if (AP_HAL::millis() - start_time_ms > startup_delay_ms) {
            _ekf3_started = EKF3.InitialiseFilter();
        }
    }
    if (_ekf3_started) {
        EKF3.UpdateFilter();
        if (_active_EKF_type() == EKFType::THREE) {
            Vector3f eulers;
            EKF3.getRotationBodyToNED(state.dcm_matrix);
            EKF3.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            const AP_InertialSensor &_ins = AP::ins();

            // Use the primary EKF to select the primary gyro
            const int8_t primary_imu = EKF3.getPrimaryCoreIMUIndex();
            const uint8_t primary_gyro = primary_imu>=0?primary_imu:_ins.get_primary_gyro();
            const uint8_t primary_accel = primary_imu>=0?primary_imu:_ins.get_primary_accel();

            // get gyro bias for primary EKF and change sign to give gyro drift
            // Note sign convention used by EKF is bias = measurement - truth
            Vector3f drift;
            EKF3.getGyroBias(-1, drift);
            state.gyro_drift = -drift;

            // use the same IMU as the primary EKF and correct for gyro drift
            state.gyro_estimate = _ins.get_gyro(primary_gyro) + state.gyro_drift;

            // get 3-axis accel bias estimates for active EKF (this is usually for the primary IMU)
            Vector3f &abias = state.accel_bias;
            EKF3.getAccelBias(-1,abias);

            // use the primary IMU for accel earth frame
            Vector3f accel = _ins.get_accel(primary_accel);
            accel -= abias;
            state.accel_ef = state.dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * accel;

            nav_filter_status filt_state;
            EKF3.getFilterStatus(filt_state);
            update_notify_from_filter_status(filt_state);
        }
    }
}
#endif

#if AP_AHRS_EXTERNAL_ENABLED
void AP_AHRS::update_external(void)
{
    external.update();
    external.get_results(external_estimates);

    if (_active_EKF_type() == EKFType::EXTERNAL) {
        copy_estimates_from_backend_estimates(external_estimates);
    }
}
#endif // AP_AHRS_EXTERNAL_ENABLED

void AP_AHRS::reset()
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);

#if AP_AHRS_DCM_ENABLED
    dcm.reset();
#endif
#if AP_AHRS_SIM_ENABLED
    sim.reset();
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    external.reset();
#endif

#if HAL_NAVEKF2_AVAILABLE
    if (_ekf2_started) {
        _ekf2_started = EKF2.InitialiseFilter();
    }
#endif
#if HAL_NAVEKF3_AVAILABLE
    if (_ekf3_started) {
        _ekf3_started = EKF3.InitialiseFilter();
    }
#endif
}

// dead-reckoning support
bool AP_AHRS::_get_location(Location &loc) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm_estimates.get_location(loc);
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (EKF2.getLLH(loc)) {
            return true;
        }
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (EKF3.getLLH(loc)) {
            return true;
        }
        break;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim_estimates.get_location(loc);
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external_estimates.get_location(loc);
#endif
    }

#if AP_AHRS_DCM_ENABLED
    // fall back to position from DCM
    if (!always_use_EKF()) {
        return dcm_estimates.get_location(loc);
    }
#endif

    return false;
}

// status reporting of estimated errors
float AP_AHRS::get_error_rp(void) const
{
#if AP_AHRS_DCM_ENABLED
    return dcm.get_error_rp();
#endif
    return 0;
}

float AP_AHRS::get_error_yaw(void) const
{
#if AP_AHRS_DCM_ENABLED
    return dcm.get_error_yaw();
#endif
    return 0;
}

// return a wind estimation vector, in m/s
bool AP_AHRS::_wind_estimate(Vector3f &wind) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.wind_estimate(wind);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.wind_estimate(wind);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getWind(wind);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getWind(wind);
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.wind_estimate(wind);
#endif
    }
    return false;
}


/*
 * Determine how aligned heading_deg is with the wind. Return result
 * is 1.0 when perfectly aligned heading into wind, -1 when perfectly
 * aligned with-wind, and zero when perfect cross-wind. There is no
 * distinction between a left or right cross-wind. Wind speed is ignored
 */
float AP_AHRS::wind_alignment(const float heading_deg) const
{
    Vector3f wind;
    if (!wind_estimate(wind)) {
        return 0;
    }
    const float wind_heading_rad = atan2f(-wind.y, -wind.x);
    return cosf(wind_heading_rad - radians(heading_deg));
}

/*
 * returns forward head-wind component in m/s. Negative means tail-wind.
 */
float AP_AHRS::head_wind(void) const
{
    const float alignment = wind_alignment(yaw_sensor*0.01f);
    return alignment * wind_estimate().xy().length();
}

/*
  return true if the current AHRS airspeed estimate is directly derived from an airspeed sensor
 */
bool AP_AHRS::using_airspeed_sensor() const
{
    return state.airspeed_estimate_type == AirspeedEstimateType::AIRSPEED_SENSOR;
}

/*
    Return true if a airspeed sensor should be used for the AHRS airspeed estimate
 */
bool AP_AHRS::_should_use_airspeed_sensor(uint8_t airspeed_index) const
{
    if (!airspeed_sensor_enabled(airspeed_index)) {
        return false;
    }
    nav_filter_status filter_status;
    if (fly_forward &&
        hal.util->get_soft_armed() &&
        get_filter_status(filter_status) &&
        (filter_status.flags.rejecting_airspeed && !filter_status.flags.dead_reckoning)) {
        // special case for when backend is rejecting airspeed data in
        // an armed fly_forward state and not dead reckoning. Then the
        // airspeed data is highly suspect and will be rejected. We
        // will use the synthetic airspeed instead
        return false;
    }
    return true;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS::_airspeed_estimate(float &airspeed_ret, AirspeedEstimateType &airspeed_estimate_type) const
{
#if AP_AHRS_DCM_ENABLED || (AP_AIRSPEED_ENABLED && AP_GPS_ENABLED)
    const uint8_t idx = get_active_airspeed_index();
#endif
#if AP_AIRSPEED_ENABLED && AP_GPS_ENABLED
    if (_should_use_airspeed_sensor(idx)) {
        airspeed_ret = AP::airspeed()->get_airspeed(idx);

        if (_wind_max > 0 && AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
            // constrain the airspeed by the ground speed
            // and AHRS_WIND_MAX
            const float gnd_speed = AP::gps().ground_speed();
            float true_airspeed = airspeed_ret * get_EAS2TAS();
            true_airspeed = constrain_float(true_airspeed,
                                            gnd_speed - _wind_max,
                                            gnd_speed + _wind_max);
            airspeed_ret = true_airspeed / get_EAS2TAS();
        }
        airspeed_estimate_type = AirspeedEstimateType::AIRSPEED_SENSOR;
        return true;
    }
#endif

    if (!get_wind_estimation_enabled()) {
        airspeed_estimate_type = AirspeedEstimateType::NO_NEW_ESTIMATE;
        return false;
    }

    // estimate it via nav velocity and wind estimates

    // get wind estimates
    Vector3f wind_vel;
    bool have_wind = false;

    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        airspeed_estimate_type = AirspeedEstimateType::DCM_SYNTHETIC;
        return dcm.airspeed_estimate(idx, airspeed_ret);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        airspeed_estimate_type = AirspeedEstimateType::SIM;
        return sim.airspeed_estimate(airspeed_ret);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#if AP_AHRS_DCM_ENABLED
        airspeed_estimate_type = AirspeedEstimateType::DCM_SYNTHETIC;
        return dcm.airspeed_estimate(idx, airspeed_ret);
#else
        return false;
#endif
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        have_wind = EKF3.getWind(wind_vel);
        break;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#if AP_AHRS_DCM_ENABLED
        airspeed_estimate_type = AirspeedEstimateType::DCM_SYNTHETIC;
        return dcm.airspeed_estimate(idx, airspeed_ret);
#else
        return false;
#endif
#endif
    }

    // estimate it via nav velocity and wind estimates
    Vector3f nav_vel;
    if (have_wind && have_inertial_nav() && get_velocity_NED(nav_vel)) {
        Vector3f true_airspeed_vec = nav_vel - wind_vel;
        float true_airspeed = true_airspeed_vec.length();
        float gnd_speed = nav_vel.length();
        if (_wind_max > 0) {
            float tas_lim_lower = MAX(0.0f, (gnd_speed - _wind_max));
            float tas_lim_upper = MAX(tas_lim_lower, (gnd_speed + _wind_max));
            true_airspeed = constrain_float(true_airspeed, tas_lim_lower, tas_lim_upper);
        } else {
            true_airspeed = MAX(0.0f, true_airspeed);
        }
        airspeed_ret = true_airspeed / get_EAS2TAS();
        airspeed_estimate_type = AirspeedEstimateType::EKF3_SYNTHETIC;
        return true;
    }

#if AP_AHRS_DCM_ENABLED
    // fallback to DCM
    airspeed_estimate_type = AirspeedEstimateType::DCM_SYNTHETIC;
    return dcm.airspeed_estimate(idx, airspeed_ret);
#endif

    return false;
}

bool AP_AHRS::_airspeed_estimate_true(float &airspeed_ret) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.airspeed_estimate_true(airspeed_ret);
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif
        break;
    }

    if (!airspeed_estimate(airspeed_ret)) {
        return false;
    }
    airspeed_ret *= get_EAS2TAS();
    return true;
}

// return estimate of true airspeed vector in body frame in m/s
// returns false if estimate is unavailable
bool AP_AHRS::_airspeed_vector_true(Vector3f &vec) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getAirSpdVec(vec);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getAirSpdVec(vec);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        break;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    return false;
}

// return the innovation in m/s, innovation variance in (m/s)^2 and age in msec of the last TAS measurement processed
// returns false if the data is unavailable
bool AP_AHRS::airspeed_health_data(float &innovation, float &innovationVariance, uint32_t &age_ms) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getAirSpdHealthData(innovation, innovationVariance, age_ms);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        break;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    return false;
}

// return a synthetic airspeed estimate (one derived from sensors
// other than an actual airspeed sensor), if available. return
// true if we have a synthetic airspeed.  ret will not be modified
// on failure.
bool AP_AHRS::synthetic_airspeed(float &ret) const
{
#if AP_AHRS_DCM_ENABLED
    return dcm.synthetic_airspeed(ret);
#endif
    return false;
}

// true if compass is being used
bool AP_AHRS::use_compass(void)
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.use_compass();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.use_compass();
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.use_compass();
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
#if AP_AHRS_DCM_ENABLED
    return dcm.use_compass();
#endif
    return false;
}

// return the quaternion defining the rotation from NED to XYZ (body) axes
bool AP_AHRS::_get_quaternion(Quaternion &quat) const
{
    // backends always return in autopilot XYZ frame; rotate result
    // according to trim
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        if (!dcm.get_quaternion(quat)) {
            return false;
        }
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (!_ekf2_started) {
            return false;
        }
        EKF2.getQuaternion(quat);
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (!_ekf3_started) {
            return false;
        }
        EKF3.getQuaternion(quat);
        break;
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        if (!sim.get_quaternion(quat)) {
            return false;
        }
        break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        // we assume the external AHRS isn't trimmed with the autopilot!
        return external.get_quaternion(quat);
#endif
    }

    quat.rotate(-_trim.get());

    return true;
}

// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS::_get_secondary_attitude(Vector3f &eulers) const
{
    EKFType secondary_ekf_type;
    if (!_get_secondary_EKF_type(secondary_ekf_type)) {
        return false;
    }

    switch (secondary_ekf_type) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // DCM is secondary
        eulers[0] = dcm_estimates.roll_rad;
        eulers[1] = dcm_estimates.pitch_rad;
        eulers[2] = dcm_estimates.yaw_rad;
        return true;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 is secondary
        EKF2.getEulerAngles(eulers);
        return _ekf2_started;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // EKF3 is secondary
        EKF3.getEulerAngles(eulers);
        return _ekf3_started;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        // SITL is secondary (should never happen)
        return false;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL: {
        // External is secondary
        eulers[0] = external_estimates.roll_rad;
        eulers[1] = external_estimates.pitch_rad;
        eulers[2] = external_estimates.yaw_rad;
        return true;
    }
#endif
    }

    // since there is no default case above, this is unreachable
    return false;
}


// return secondary attitude solution if available, as quaternion
bool AP_AHRS::_get_secondary_quaternion(Quaternion &quat) const
{
    EKFType secondary_ekf_type;
    if (!_get_secondary_EKF_type(secondary_ekf_type)) {
        return false;
    }

    switch (secondary_ekf_type) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // DCM is secondary
        if (!dcm.get_quaternion(quat)) {
            return false;
        }
        break;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 is secondary
        if (!_ekf2_started) {
            return false;
        }
        EKF2.getQuaternion(quat);
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // EKF3 is secondary
        if (!_ekf3_started) {
            return false;
        }
        EKF3.getQuaternion(quat);
        break;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        // SITL is secondary (should never happen)
        return false;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        // External is secondary
        return external.get_quaternion(quat);
#endif
    }

    quat.rotate(-_trim.get());

    return true;
}

// return secondary position solution if available
bool AP_AHRS::_get_secondary_position(Location &loc) const
{
    EKFType secondary_ekf_type;
    if (!_get_secondary_EKF_type(secondary_ekf_type)) {
        return false;
    }

    switch (secondary_ekf_type) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // return DCM position
        loc = dcm_estimates.location;
        // FIXME: we intentionally do not return whether location is
        // actually valid here so we continue to send mavlink messages
        // and log data:
        return true;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 is secondary
        EKF2.getLLH(loc);
        return _ekf2_started;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // EKF3 is secondary
        EKF3.getLLH(loc);
        return _ekf3_started;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        // SITL is secondary (should never happen)
        return false;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        // External is secondary
        return external_estimates.get_location(loc);
#endif
    }

    // since there is no default case above, this is unreachable
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS::_groundspeed_vector(void)
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.groundspeed_vector();
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        Vector3f vec;
        EKF2.getVelNED(vec);
        return vec.xy();
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        Vector3f vec;
        EKF3.getVelNED(vec);
        return vec.xy();
    }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.groundspeed_vector();
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL: {
        return external.groundspeed_vector();
    }
#endif
    }
    return Vector2f();
}

float AP_AHRS::_groundspeed(void)
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.groundspeed();
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
        break;
    }
    return groundspeed_vector().length();
}

// set the EKF's origin location in 10e7 degrees.  This should only
// be called when the EKF has no absolute position reference (i.e. GPS)
// from which to decide the origin on its own
bool AP_AHRS::set_origin(const Location &loc)
{
    WITH_SEMAPHORE(_rsem);
#if HAL_NAVEKF2_AVAILABLE
    const bool ret2 = EKF2.setOriginLLH(loc);
#endif
#if HAL_NAVEKF3_AVAILABLE
    const bool ret3 = EKF3.setOriginLLH(loc);
#endif

    // return success if active EKF's origin was set
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return ret2;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return ret3;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        // never allow origin set in SITL. The origin is set by the
        // simulation backend
        return false;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        // don't allow origin set with external AHRS
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

#if AP_AHRS_POSITION_RESET_ENABLED
bool AP_AHRS::handle_external_position_estimate(const Location &loc, float pos_accuracy, uint32_t timestamp_ms)
{
#if HAL_NAVEKF3_AVAILABLE
    return EKF3.setLatLng(loc, pos_accuracy, timestamp_ms);
#endif
    return false;
}
#endif

// return true if inertial navigation is active
bool AP_AHRS::have_inertial_nav(void) const
{
#if AP_AHRS_DCM_ENABLED
    return active_EKF_type() != EKFType::DCM;
#endif
    return true;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS::_get_velocity_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getVelNED(vec);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getVelNED(vec);
        return true;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_velocity_NED(vec);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.get_velocity_NED(vec);
#endif
    }
#if AP_AHRS_DCM_ENABLED
    return dcm.get_velocity_NED(vec);
#endif
    return false;
}

// returns the expected NED magnetic field
bool AP_AHRS::get_mag_field_NED(Vector3f &vec) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return false;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getMagNED(vec);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getMagNED(vec);
        return true;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return false;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    return false;
}

// returns the estimated magnetic field offsets in body frame
bool AP_AHRS::get_mag_field_correction(Vector3f &vec) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return false;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getMagXYZ(vec);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getMagXYZ(vec);
        return true;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return false;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
// This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
bool AP_AHRS::get_vert_pos_rate_D(float &velocity) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.get_vert_pos_rate_D(velocity);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        velocity = EKF2.getPosDownDerivative();
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        velocity = EKF3.getPosDownDerivative();
        return true;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_vert_pos_rate_D(velocity);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.get_vert_pos_rate_D(velocity);
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// get latest height above ground level estimate in metres and a validity flag
bool AP_AHRS::get_hagl(float &height) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return false;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getHAGL(height);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getHAGL(height);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_hagl(height);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL: {
        return false;
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

/*
  return a relative NED position from the origin in meters
*/
bool AP_AHRS::get_relative_position_NED_origin(Vector3f &vec) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.get_relative_position_NED_origin(vec);
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        Vector2f posNE;
        float posD;
        if (EKF2.getPosNE(posNE) && EKF2.getPosD(posD)) {
            // position is valid
            vec.x = posNE.x;
            vec.y = posNE.y;
            vec.z = posD;
            return true;
        }
        return false;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
            Vector2f posNE;
            float posD;
            if (EKF3.getPosNE(posNE) && EKF3.getPosD(posD)) {
                // position is valid
                vec.x = posNE.x;
                vec.y = posNE.y;
                vec.z = posD;
                return true;
            }
            return false;
        }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_relative_position_NED_origin(vec);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL: {
        return external.get_relative_position_NED_origin(vec);
    }
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

/*
 return a relative ground position from home in meters
*/
bool AP_AHRS::get_relative_position_NED_home(Vector3f &vec) const
{
    Location loc;
    if (!_home_is_set ||
        !get_location(loc)) {
        return false;
    }
    vec = _home.get_distance_NED(loc);
    return true;
}

/*
  return a relative position estimate from the origin in meters
*/
bool AP_AHRS::get_relative_position_NE_origin(Vector2f &posNE) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.get_relative_position_NE_origin(posNE);
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        bool position_is_valid = EKF2.getPosNE(posNE);
        return position_is_valid;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        bool position_is_valid = EKF3.getPosNE(posNE);
        return position_is_valid;
    }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM: {
        return sim.get_relative_position_NE_origin(posNE);
    }
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.get_relative_position_NE_origin(posNE);
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

/*
 return a relative ground position from home in meters North/East
*/
bool AP_AHRS::get_relative_position_NE_home(Vector2f &posNE) const
{
    Location loc;
    if (!_home_is_set ||
        !get_location(loc)) {
        return false;
    }

    posNE = _home.get_distance_NE(loc);
    return true;
}

// write a relative ground position estimate to the origin in meters, North/East order


/*
  return a relative ground position from the origin in meters, down
*/
bool AP_AHRS::get_relative_position_D_origin(float &posD) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.get_relative_position_D_origin(posD);
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        bool position_is_valid = EKF2.getPosD(posD);
        return position_is_valid;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        bool position_is_valid = EKF3.getPosD(posD);
        return position_is_valid;
    }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_relative_position_D_origin(posD);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.get_relative_position_D_origin(posD);
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

/*
  return relative position from home in meters
*/
void AP_AHRS::get_relative_position_D_home(float &posD) const
{
    if (!_home_is_set) {
        // fall back to an altitude derived from barometric pressure
        // differences vs a calibrated ground pressure:
        posD = -AP::baro().get_altitude();
        return;
    }

    Location originLLH;
    float originD;
    if (!get_relative_position_D_origin(originD) ||
        !_get_origin(originLLH)) {
#if AP_GPS_ENABLED
        const auto &gps = AP::gps();
        if (_gps_use == GPSUse::EnableWithHeight &&
            gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            posD = (get_home().alt - gps.location().alt) * 0.01;
            return;
        }
#endif
        posD = -AP::baro().get_altitude();
        return;
    }

    posD = originD - ((originLLH.alt - _home.alt) * 0.01f);
    return;
}
/*
  canonicalise _ekf_type, forcing it to be 0, 2 or 3
  type 1 has been deprecated
 */
AP_AHRS::EKFType AP_AHRS::ekf_type(void) const
{
    EKFType type = (EKFType)_ekf_type.get();
    switch (type) {
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return type;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return type;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return type;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return type;
#endif
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        if (always_use_EKF()) {
#if HAL_NAVEKF2_AVAILABLE
            return EKFType::TWO;
#elif HAL_NAVEKF3_AVAILABLE
            return EKFType::THREE;
#endif
        }
        return EKFType::DCM;
#endif
    }
    // we can get to here if the user has mis-set AHRS_EKF_TYPE - any
    // value above 3 will get to here.  TWO is returned here for no
    // better reason than "tradition".
#if HAL_NAVEKF2_AVAILABLE
    return EKFType::TWO;
#elif HAL_NAVEKF3_AVAILABLE
    return EKFType::THREE;
#elif AP_AHRS_DCM_ENABLED
    return EKFType::DCM;
#else
    #error "no default backend available"
#endif
}

AP_AHRS::EKFType AP_AHRS::_active_EKF_type(void) const
{
    EKFType ret = fallback_active_EKF_type();

    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return EKFType::DCM;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        // do we have an EKF2 yet?
        if (!_ekf2_started) {
            return fallback_active_EKF_type();
        }
        if (always_use_EKF()) {
            uint16_t ekf2_faults;
            EKF2.getFilterFaults(ekf2_faults);
            if (ekf2_faults == 0) {
                ret = EKFType::TWO;
            }
        } else if (EKF2.healthy()) {
            ret = EKFType::TWO;
        }
        break;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        // do we have an EKF3 yet?
        if (!_ekf3_started) {
            return fallback_active_EKF_type();
        }
        if (always_use_EKF()) {
            uint16_t ekf3_faults;
            EKF3.getFilterFaults(ekf3_faults);
            if (ekf3_faults == 0) {
                ret = EKFType::THREE;
            }
        } else if (EKF3.healthy()) {
            ret = EKFType::THREE;
        }
        break;
    }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        ret = EKFType::SIM;
        break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        ret = EKFType::EXTERNAL;
        break;
#endif
    }

#if AP_AHRS_DCM_ENABLED
    // Handle fallback for fixed wing planes (including VTOL's) and ground vehicles.
    if (_vehicle_class == VehicleClass::FIXED_WING ||
        _vehicle_class == VehicleClass::GROUND) {
        bool should_use_gps = true;
        nav_filter_status filt_state {};
        switch (ret) {
        case EKFType::DCM:
            // already using DCM
            break;
#if HAL_NAVEKF2_AVAILABLE
        case EKFType::TWO:
            EKF2.getFilterStatus(filt_state);
            should_use_gps = EKF2.configuredToUseGPSForPosXY();
            break;
#endif
#if HAL_NAVEKF3_AVAILABLE
        case EKFType::THREE:
            EKF3.getFilterStatus(filt_state);
            should_use_gps = EKF3.configuredToUseGPSForPosXY();
            break;
#endif
#if AP_AHRS_SIM_ENABLED
        case EKFType::SIM:
            get_filter_status(filt_state);
            break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
        case EKFType::EXTERNAL:
            get_filter_status(filt_state);
            should_use_gps = true;
            break;
#endif
        }

        // Handle fallback for the case where the DCM or EKF is unable to provide attitude or height data.
        const bool can_use_dcm = dcm.yaw_source_available() || fly_forward;
        const bool can_use_ekf = filt_state.flags.attitude && filt_state.flags.vert_vel && filt_state.flags.vert_pos;
        if (!can_use_dcm && can_use_ekf) {
            // no choice - continue to use EKF
            return ret;
        } else if (!can_use_ekf) {
            // No choice - we have to use DCM
            return EKFType::DCM;
        }

        const bool disable_dcm_fallback = fly_forward?
            option_set(Options::DISABLE_DCM_FALLBACK_FW) : option_set(Options::DISABLE_DCM_FALLBACK_VTOL);
        if (disable_dcm_fallback) {
            // don't fallback
            return ret;
        }
        
        // Handle loss of global position when we still have a GPS fix
        if (hal.util->get_soft_armed() &&
            (_gps_use != GPSUse::Disable) &&
            should_use_gps &&
            AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D &&
            (!filt_state.flags.using_gps || !filt_state.flags.horiz_pos_abs)) {
            /*
               If the EKF is not fusing GPS or doesn't have a 2D fix and we have a 3D GPS lock,
               then plane and rover would prefer to use the GPS position from DCM unless the
               fallback has been inhibited by the user.
               Note: The aircraft could be dead reckoning with acceptable accuracy and rejecting a bad GPS
               Note: This is a last resort fallback and makes the navigation highly vulnerable to GPS noise.
               Note: When operating in a VTOL flight mode that actively controls height such as QHOVER,
               the EKF gives better vertical velocity and position estimates and height control characteristics.
            */
            return EKFType::DCM;
        }

        // Handle complete loss of navigation
        if (hal.util->get_soft_armed() && filt_state.flags.const_pos_mode) {
            /*
               Provided the EKF has been configured to use GPS, ie should_use_gps is true, then the
               key difference to the case handled above is only the absence of a GPS fix which means
               that DCM will not be able to navigate either so we are primarily concerned with
               providing an attitude, vertical position and vertical velocity estimate.
            */
            return EKFType::DCM;
        }

        if (!filt_state.flags.horiz_vel ||
            (!filt_state.flags.horiz_pos_abs && !filt_state.flags.horiz_pos_rel)) {
            if ((!AP::compass().use_for_yaw()) &&
                AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D &&
                AP::gps().ground_speed() < 2) {
                /*
                  special handling for non-compass mode when sitting
                  still. The EKF may not yet have aligned its yaw. We
                  accept EKF as healthy to allow arming. Once we reach
                  speed the EKF should get yaw alignment
                */
                if (filt_state.flags.gps_quality_good) {
                    return ret;
                }
            }
            return EKFType::DCM;
        }
    }
#endif

    return ret;
}

AP_AHRS::EKFType AP_AHRS::fallback_active_EKF_type(void) const
{
#if AP_AHRS_DCM_ENABLED
    return EKFType::DCM;
#endif

#if HAL_NAVEKF3_AVAILABLE
    if (_ekf3_started) {
        return EKFType::THREE;
    }
#endif

#if HAL_NAVEKF2_AVAILABLE
    if (_ekf2_started) {
        return EKFType::TWO;
    }
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    if (external.healthy()) {
        return EKFType::EXTERNAL;
    }
#endif

    // so nobody is ready yet.  Return something, even if it is not ready:
#if HAL_NAVEKF3_AVAILABLE
    return EKFType::THREE;
#elif HAL_NAVEKF2_AVAILABLE
    return EKFType::TWO;
#elif AP_AHRS_EXTERNAL_ENABLED
    return EKFType::EXTERNAL;
#endif
}

// get secondary EKF type.  returns false if no secondary (i.e. only using DCM)
bool AP_AHRS::_get_secondary_EKF_type(EKFType &secondary_ekf_type) const
{

    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // EKF2, EKF3 or External is secondary
#if HAL_NAVEKF3_AVAILABLE
        if ((EKFType)_ekf_type.get() == EKFType::THREE) {
            secondary_ekf_type = EKFType::THREE;
            return true;
        }
#endif
#if HAL_NAVEKF2_AVAILABLE
        if ((EKFType)_ekf_type.get() == EKFType::TWO) {
            secondary_ekf_type = EKFType::TWO;
            return true;
        }
#endif
#if AP_AHRS_EXTERNAL_ENABLED
        if ((EKFType)_ekf_type.get() == EKFType::EXTERNAL) {
            secondary_ekf_type = EKFType::EXTERNAL;
            return true;
        }
#endif
        return false;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif
        // DCM is secondary
        secondary_ekf_type = fallback_active_EKF_type();
        return true;
    }

    // since there is no default case above, this is unreachable
    return false;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS::healthy(void) const
{
    // If EKF is started we switch away if it reports unhealthy. This could be due to bad
    // sensor data. If EKF reversion is inhibited, we only switch across if the EKF encounters
    // an internal processing error, but not for bad sensor data.
    switch (ekf_type()) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.healthy();
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        bool ret = _ekf2_started && EKF2.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == VehicleClass::FIXED_WING ||
                _vehicle_class == VehicleClass::GROUND) &&
                active_EKF_type() != EKFType::TWO) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        bool ret = _ekf3_started && EKF3.healthy();
        if (!ret) {
            return false;
        }
        if ((_vehicle_class == VehicleClass::FIXED_WING ||
                _vehicle_class == VehicleClass::GROUND) &&
                active_EKF_type() != EKFType::THREE) {
            // on fixed wing we want to be using EKF to be considered
            // healthy if EKF is enabled
            return false;
        }
        return true;
    }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.healthy();
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.healthy();
#endif
    }

    return false;
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
// requires_position should be true if horizontal position configuration should be checked
bool AP_AHRS::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    bool ret = true;
    if (!healthy()) {
        // this rather generic failure might be overwritten by
        // something more specific in the "backend"
        hal.util->snprintf(failure_msg, failure_msg_len, "Not healthy");
        ret = false;
    }

#if AP_AHRS_EXTERNAL_ENABLED
    // Always check external AHRS if enabled
    // it is a source for IMU data even if not being used as direct AHRS replacement
    if (AP::externalAHRS().enabled() || (ekf_type() == EKFType::EXTERNAL)) {
        if (!AP::externalAHRS().pre_arm_check(failure_msg, failure_msg_len)) {
            return false;
        }
    }
#endif

    if (!attitudes_consistent(failure_msg, failure_msg_len)) {
        return false;
    }

    // ensure we're using the configured backend, but bypass in compass-less cases:
    if (ekf_type() != active_EKF_type() && AP::compass().use_for_yaw()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "not using configured AHRS type");
        return false;
    }

    switch (ekf_type()) {
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return ret;
#endif

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.pre_arm_check(requires_position, failure_msg, failure_msg_len) && ret;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.pre_arm_check(requires_position, failure_msg, failure_msg_len);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        if (!_ekf2_started) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 not started");
            return false;
        }
        return EKF2.pre_arm_check(failure_msg, failure_msg_len) && ret;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        if (!_ekf3_started) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF3 not started");
            return false;
        }
        return EKF3.pre_arm_check(requires_position, failure_msg, failure_msg_len) && ret;
#endif
    }

    // if we get here then ekf type is invalid
    hal.util->snprintf(failure_msg, failure_msg_len, "invalid EKF type");
    return false;
}

// true if the AHRS has completed initialisation
bool AP_AHRS::initialised(void) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return true;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // initialisation complete 10sec after ekf has started
        return (_ekf2_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // initialisation complete 10sec after ekf has started
        return (_ekf3_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return true;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.initialised();
#endif
    }
    return false;
};

// get_filter_status : returns filter status as a series of flags
bool AP_AHRS::get_filter_status(nav_filter_status &status) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.get_filter_status(status);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getFilterStatus(status);
        return true;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getFilterStatus(status);
        return true;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_filter_status(status);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.get_filter_status(status);
#endif
    }

    return false;
}

// write optical flow data to EKF
void  AP_AHRS::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset, heightOverride);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset, heightOverride);
#endif
}

// retrieve latest corrected optical flow samples (used for calibration)
bool AP_AHRS::getOptFlowSample(uint32_t& timeStamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const
{
#if HAL_NAVEKF3_AVAILABLE
    return EKF3.getOptFlowSample(timeStamp_ms, flowRate, bodyRate, losPred);
#endif
    return false;
}

// write body frame odometry measurements to the EKF
void  AP_AHRS::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset)
{
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset);
#endif
}

// Write position and quaternion data from an external navigation system
void AP_AHRS::writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
#endif
}

// Writes the default equivalent airspeed and 1-sigma uncertainty in m/s to be used in forward flight if a measured airspeed is required and not available.
void AP_AHRS::writeDefaultAirSpeed(float airspeed, float uncertainty)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeDefaultAirSpeed(airspeed);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeDefaultAirSpeed(airspeed, uncertainty);
#endif
}

// Write velocity data from an external navigation system
void AP_AHRS::writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
#endif
}

// get speed limit and XY navigation gain scale factor
void AP_AHRS::getControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        dcm.get_control_limits(ekfGndSpdLimit, ekfNavVelGainScaler);
        break;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.getEkfControlLimits(ekfGndSpdLimit,ekfNavVelGainScaler);
        break;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        sim.get_control_limits(ekfGndSpdLimit, ekfNavVelGainScaler);
        break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        // no limit on gains, large vel limit
        ekfGndSpdLimit = 400;
        ekfNavVelGainScaler = 1;
        break;
#endif
    }
}

/*
  get gain factor for Z controllers
 */
float AP_AHRS::getControlScaleZ(void) const
{
#if AP_AHRS_DCM_ENABLED
    if (active_EKF_type() == EKFType::DCM) {
        // when flying on DCM lower gains by 4x to cope with the high
        // lag
        return 0.25;
    }
#endif
    return 1;
}

// get compass offset estimates
// true if offsets are valid
bool AP_AHRS::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return false;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getMagOffsets(mag_idx, magOffsets);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getMagOffsets(mag_idx, magOffsets);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_mag_offsets(mag_idx, magOffsets);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    // since there is no default case above, this is unreachable
    return false;
}

// Retrieves the NED delta velocity corrected
void AP_AHRS::_getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const
{
    int8_t imu_idx = -1;
    Vector3f accel_bias;
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        imu_idx = EKF2.getPrimaryCoreIMUIndex();
        EKF2.getAccelZBias(accel_bias.z);
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        imu_idx = EKF3.getPrimaryCoreIMUIndex();
        EKF3.getAccelBias(-1,accel_bias);
        break;
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    ret.zero();
    if (imu_idx == -1) {
        AP::ins().get_delta_velocity(ret, dt);
        return;
    }
    AP::ins().get_delta_velocity((uint8_t)imu_idx, ret, dt);
    ret -= accel_bias*dt;
    ret = state.dcm_matrix * get_rotation_autopilot_body_to_vehicle_body() * ret;
    ret.z += GRAVITY_MSS*dt;
}

void AP_AHRS::set_failure_inconsistent_message(const char *estimator, const char *axis, float diff_rad, char *failure_msg, const uint8_t failure_msg_len) const
{
    hal.util->snprintf(failure_msg, failure_msg_len, "%s %s inconsistent %d deg. Wait or reboot", estimator, axis, (int)degrees(diff_rad));
}

// check all cores providing consistent attitudes for prearm checks
bool AP_AHRS::attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const
{
    // get primary attitude source's attitude as quaternion
    Quaternion primary_quat;
    get_quat_body_to_ned(primary_quat);
    // only check yaw if compasses are being used
    const bool check_yaw = AP::compass().use_for_yaw();
    uint8_t total_ekf_cores = 0;

#if HAL_NAVEKF2_AVAILABLE
    // check primary vs ekf2
    if (ekf_type() == EKFType::TWO || active_EKF_type() == EKFType::TWO) {
        for (uint8_t i = 0; i < EKF2.activeCores(); i++) {
            Quaternion ekf2_quat;
            EKF2.getQuaternionBodyToNED(i, ekf2_quat);

            // check roll and pitch difference
            const float rp_diff_rad = primary_quat.roll_pitch_difference(ekf2_quat);
            if (rp_diff_rad > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
                set_failure_inconsistent_message("EKF2", "Roll/Pitch", rp_diff_rad, failure_msg, failure_msg_len);
                return false;
            }

            // check yaw difference
            Vector3f angle_diff;
            primary_quat.angular_difference(ekf2_quat).to_axis_angle(angle_diff);
            const float yaw_diff = fabsf(angle_diff.z);
            if (check_yaw && (yaw_diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
                set_failure_inconsistent_message("EKF2", "Yaw", yaw_diff, failure_msg, failure_msg_len);
                return false;
            }
        }
        total_ekf_cores = EKF2.activeCores();
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    // check primary vs ekf3
    if (ekf_type() == EKFType::THREE || active_EKF_type() == EKFType::THREE) {
        for (uint8_t i = 0; i < EKF3.activeCores(); i++) {
            Quaternion ekf3_quat;
            EKF3.getQuaternionBodyToNED(i, ekf3_quat);

            // check roll and pitch difference
            const float rp_diff_rad = primary_quat.roll_pitch_difference(ekf3_quat);
            if (rp_diff_rad > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
                set_failure_inconsistent_message("EKF3", "Roll/Pitch", rp_diff_rad, failure_msg, failure_msg_len);
                return false;
            }

            // check yaw difference
            Vector3f angle_diff;
            primary_quat.angular_difference(ekf3_quat).to_axis_angle(angle_diff);
            const float yaw_diff = fabsf(angle_diff.z);
            if (check_yaw && (yaw_diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
                set_failure_inconsistent_message("EKF3", "Yaw", yaw_diff, failure_msg, failure_msg_len);
                return false;
            }
        }
        total_ekf_cores += EKF3.activeCores();
    }
#endif

#if AP_AHRS_DCM_ENABLED
    // check primary vs dcm
    if (!always_use_EKF() || (total_ekf_cores == 1)) {
        Quaternion dcm_quat;
        dcm_quat.from_rotation_matrix(get_DCM_rotation_body_to_ned());

        // check roll and pitch difference
        const float rp_diff_rad = primary_quat.roll_pitch_difference(dcm_quat);
        if (rp_diff_rad > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
            set_failure_inconsistent_message("DCM", "Roll/Pitch", rp_diff_rad, failure_msg, failure_msg_len);
            return false;
        }

        // Check vs DCM yaw if this vehicle could use DCM in flight
        // and if not using an external yaw source (DCM does not support external yaw sources)
        bool using_noncompass_for_yaw = false;
#if HAL_NAVEKF3_AVAILABLE
        using_noncompass_for_yaw = (ekf_type() == EKFType::THREE) && EKF3.using_noncompass_for_yaw();
#endif
        if (!always_use_EKF() && !using_noncompass_for_yaw) {
            Vector3f angle_diff;
            primary_quat.angular_difference(dcm_quat).to_axis_angle(angle_diff);
            const float yaw_diff = fabsf(angle_diff.z);
            if (check_yaw && (yaw_diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
                set_failure_inconsistent_message("DCM", "Yaw", yaw_diff, failure_msg, failure_msg_len);
                return false;
            }
        }
    }
#endif

    return true;
}

// return the amount of yaw angle change due to the last yaw angle reset in radians
// returns the time of the last yaw angle reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastYawResetAngle(float &yawAng)
{
    switch (active_EKF_type()) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.getLastYawResetAngle(yawAng);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastYawResetAngle(yawAng);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastYawResetAngle(yawAng);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.getLastYawResetAngle(yawAng);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.getLastYawResetAngle(yawAng);
#endif
    }
    return 0;
}

// return the amount of NE position change in metres due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastPosNorthEastReset(Vector2f &pos)
{
    switch (active_EKF_type()) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return 0;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastPosNorthEastReset(pos);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastPosNorthEastReset(pos);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.getLastPosNorthEastReset(pos);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return 0;
#endif
    }
    return 0;
}

// return the amount of NE velocity change in metres/sec due to the last reset
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastVelNorthEastReset(Vector2f &vel) const
{
    switch (active_EKF_type()) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return 0;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastVelNorthEastReset(vel);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastVelNorthEastReset(vel);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.getLastVelNorthEastReset(vel);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return 0;
#endif
    }
    return 0;
}


// return the amount of vertical position change due to the last reset in meters
// returns the time of the last reset or 0 if no reset has ever occurred
uint32_t AP_AHRS::getLastPosDownReset(float &posDelta)
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return 0;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getLastPosDownReset(posDelta);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getLastPosDownReset(posDelta);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.getLastPosDownReset(posDelta);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return 0;
#endif
    }
    return 0;
}

// Resets the baro so that it reads zero at the current height
// Resets the EKF height to zero
// Adjusts the EKf origin height so that the EKF height + origin height is the same as before
// Returns true if the height datum reset has been performed
// If using a range finder for height no reset is performed and it returns false
bool AP_AHRS::resetHeightDatum(void)
{
    // support locked access functions to AHRS data
    WITH_SEMAPHORE(_rsem);
    
    switch (ekf_type()) {

#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
#if HAL_NAVEKF3_AVAILABLE
        EKF3.resetHeightDatum();
#endif
#if HAL_NAVEKF2_AVAILABLE
        EKF2.resetHeightDatum();
#endif
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#if HAL_NAVEKF3_AVAILABLE
        EKF3.resetHeightDatum();
#endif
        return EKF2.resetHeightDatum();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
#if HAL_NAVEKF2_AVAILABLE
        EKF2.resetHeightDatum();
#endif
        return EKF3.resetHeightDatum();
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.resetHeightDatum();
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }
    return false;
}

// send a EKF_STATUS_REPORT for configured EKF
void AP_AHRS::send_ekf_status_report(GCS_MAVLINK &link) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // send zero status report
        dcm.send_ekf_status_report(link);
        break;
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        sim.send_ekf_status_report(link);
        break;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL: {
        external.send_ekf_status_report(link);
        break;
    }
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.send_status_report(link);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.send_status_report(link);
#endif

    }
}

// return origin for a specified EKF type
bool AP_AHRS::_get_origin(EKFType type, Location &ret) const
{
    switch (type) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        return dcm.get_origin(ret);
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getOriginLLH(ret);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getOriginLLH(ret);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_origin(ret);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return external.get_origin(ret);
#endif
    }
    return false;
}

/*
  return origin for the configured EKF type. If we are armed and the
  configured EKF type cannot return an origin then return origin for
  the active EKF type (if available)

  This copes with users force arming a plane that is running on DCM as
  the EKF has not fully initialised
 */
bool AP_AHRS::_get_origin(Location &ret) const
{
    if (_get_origin(ekf_type(), ret)) {
        return true;
    }
    if (hal.util->get_soft_armed() && _get_origin(active_EKF_type(), ret)) {
        return true;
    }
    return false;
}

bool AP_AHRS::set_home(const Location &loc)
{
    WITH_SEMAPHORE(_rsem);
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0 && loc.alt == 0) {
        return false;
    }
    if (!loc.check_latlng()) {
        return false;
    }
    // home must always be global frame at the moment as .alt is
    // accessed directly by the vehicles and they may not be rigorous
    // in checking the frame type.
    Location tmp = loc;
    if (!tmp.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return false;
    }

#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN) && HAL_LOGGING_ENABLED
    if (!_home_is_set) {
        // record home is set
        AP::logger().Write_Event(LogEvent::SET_HOME);
    }
#endif

    _home = tmp;
    _home_is_set = true;

#if HAL_LOGGING_ENABLED
    Log_Write_Home_And_Origin();
#endif

    // send new home and ekf origin to GCS
    GCS_SEND_MESSAGE(MSG_HOME);
    GCS_SEND_MESSAGE(MSG_ORIGIN);

    AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    pd.home_lat = loc.lat;
    pd.home_lon = loc.lng;
    pd.home_alt_cm = loc.alt;

    return true;
}

/* if this was a watchdog reset then get home from backup registers */
void AP_AHRS::load_watchdog_home()
{
    const AP_HAL::Util::PersistentData &pd = hal.util->persistent_data;
    if (hal.util->was_watchdog_reset() && (pd.home_lat != 0 || pd.home_lon != 0)) {
        _home.lat = pd.home_lat;
        _home.lng = pd.home_lon;
        _home.set_alt_cm(pd.home_alt_cm, Location::AltFrame::ABSOLUTE);
        _home_is_set = true;
        _home_locked = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Restored watchdog home");
    }
}

// get_hgt_ctrl_limit - get maximum height to be observed by the control loops in metres and a validity flag
// this is used to limit height during optical flow navigation
// it will return false when no limiting is required
bool AP_AHRS::get_hgt_ctrl_limit(float& limit) const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // We are not using an EKF so no limiting applies
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getHeightControlLimit(limit);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getHeightControlLimit(limit);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_hgt_ctrl_limit(limit);
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// this is not related to terrain following
void AP_AHRS::set_terrain_hgt_stable(bool stable)
{
    // avoid repeatedly setting variable in NavEKF objects to prevent
    // spurious event logging
    switch (terrainHgtStableState) {
    case TriState::UNKNOWN:
        break;
    case TriState::True:
        if (stable) {
            return;
        }
        break;
    case TriState::False:
        if (!stable) {
            return;
        }
        break;
    }
    terrainHgtStableState = (TriState)stable;

#if HAL_NAVEKF2_AVAILABLE
    EKF2.setTerrainHgtStable(stable);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.setTerrainHgtStable(stable);
#endif
}

// return the innovations for the primarily EKF
// boolean false is returned if innovations are not available
bool AP_AHRS::get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // We are not using an EKF so no data
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // use EKF to get innovations
        return EKF2.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // use EKF to get innovations
        return EKF3.getInnovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_innovations(velInnov, posInnov, magInnov, tasInnov, yawInnov);
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

// returns true when the state estimates are significantly degraded by vibration
bool AP_AHRS::is_vibration_affected() const
{
    switch (ekf_type()) {
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.isVibrationAffected();
#endif
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif
        return false;
    }
    return false;
}

// get_variances - provides the innovations normalised using the innovation variance where a value of 0
// indicates prefect consistency between the measurement and the EKF solution and a value of 1 is the maximum
// inconsistency that will be accepted by the filter
// boolean false is returned if variances are not available
bool AP_AHRS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // We are not using an EKF so no data
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO: {
        // use EKF to get variance
        Vector2f offset;
        return EKF2.getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE: {
        // use EKF to get variance
        Vector2f offset;
        return EKF3.getVariances(velVar, posVar, hgtVar, magVar, tasVar, offset);
    }
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        return sim.get_variances(velVar, posVar, hgtVar, magVar, tasVar);
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

// get a source's velocity innovations.  source should be from 0 to 7 (see AP_NavEKF_Source::SourceXY)
// returns true on success and results are placed in innovations and variances arguments
bool AP_AHRS::get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const
{
    switch (ekf_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // We are not using an EKF so no data
        return false;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // EKF2 does not support source level variances
        return false;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // use EKF to get variance
        return EKF3.getVelInnovationsAndVariancesForSource((AP_NavEKF_Source::SourceXY)source, innovations, variances);
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        // SITL does not support source level variances
        return false;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        return false;
#endif
    }

    return false;
}

//get the index of the active airspeed sensor, wrt the primary core
uint8_t AP_AHRS::get_active_airspeed_index() const
{
#if AP_AIRSPEED_ENABLED
    const auto *airspeed = AP::airspeed();
    if (airspeed == nullptr) {
        return 0;
    }

// we only have affinity for EKF3 as of now
#if HAL_NAVEKF3_AVAILABLE
    if (active_EKF_type() == EKFType::THREE) {
        uint8_t ret = EKF3.getActiveAirspeed();
        if (ret != 255 && airspeed->healthy(ret) && airspeed->use(ret)) {
            return ret;
        }
    }
#endif

    // for the rest, let the primary airspeed sensor be used
    return airspeed->get_primary();
#else

    return 0;
#endif // AP_AIRSPEED_ENABLED
}

// get the index of the current primary IMU
uint8_t AP_AHRS::_get_primary_IMU_index() const
{
    int8_t imu = -1;
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        // let EKF2 choose primary IMU
        imu = EKF2.getPrimaryCoreIMUIndex();
        break;
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        // let EKF2 choose primary IMU
        imu = EKF3.getPrimaryCoreIMUIndex();
        break;
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        break;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
    }
    if (imu == -1) {
        imu = AP::ins().get_primary_gyro();
    }
    return imu;
}

// return the index of the primary core or -1 if no primary core selected
int8_t AP_AHRS::_get_primary_core_index() const
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        // we have only one core
        return 0;
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        // we have only one core
        return 0;
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        // we have only one core
        return 0;
#endif

#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.getPrimaryCoreIndex();
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.getPrimaryCoreIndex();
#endif
    }

    // we should never get here
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return -1;
}

// get the index of the current primary accelerometer sensor
uint8_t AP_AHRS::_get_primary_accel_index(void) const
{
    return _get_primary_IMU_index();
}

// get the index of the current primary gyro sensor
uint8_t AP_AHRS::_get_primary_gyro_index(void) const
{
    return _get_primary_IMU_index();
}

// see if EKF lane switching is possible to avoid EKF failsafe
void AP_AHRS::check_lane_switch(void)
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif

#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        break;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
        
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.checkLaneSwitch();
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.checkLaneSwitch();
        break;
#endif
    }
}

// request EKF yaw reset to try and avoid the need for an EKF lane switch or failsafe
void AP_AHRS::request_yaw_reset(void)
{
    switch (active_EKF_type()) {
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
        break;
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
        break;
#endif

#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
        break;
#endif
        
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        EKF2.requestYawReset();
        break;
#endif

#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        EKF3.requestYawReset();
        break;
#endif
    }
}

// set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
void AP_AHRS::set_posvelyaw_source_set(uint8_t source_set_idx)
{
#if HAL_NAVEKF3_AVAILABLE
    EKF3.setPosVelYawSourceSet(source_set_idx);
#endif
}

//returns active source set used, 0=primary, 1=secondary, 2=tertiary
uint8_t AP_AHRS::get_posvelyaw_source_set() const
{
#if HAL_NAVEKF3_AVAILABLE
    return EKF3.get_active_source_set();
#else
    return 0;
#endif   
}

void AP_AHRS::Log_Write()
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.Log_Write();
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.Log_Write();
#endif

    Write_AHRS2();
    Write_POS();

#if AP_AHRS_SIM_ENABLED
    AP::sitl()->Log_Write_SIMSTATE();
#endif
}

// check if non-compass sensor is providing yaw.  Allows compass pre-arm checks to be bypassed
bool AP_AHRS::using_noncompass_for_yaw(void) const
{
    switch (active_EKF_type()) {
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.isExtNavUsedForYaw();
#endif
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.using_noncompass_for_yaw();
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif
        return false; 
    }
    // since there is no default case above, this is unreachable
    return false;
}

// check if external nav is providing yaw
bool AP_AHRS::using_extnav_for_yaw(void) const
{
    switch (active_EKF_type()) {
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.isExtNavUsedForYaw();
#endif
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.using_extnav_for_yaw();
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif
        return false;
    }
    // since there is no default case above, this is unreachable
    return false;
}

// set and save the alt noise parameter value
void AP_AHRS::set_alt_measurement_noise(float noise)
{
#if HAL_NAVEKF2_AVAILABLE
    EKF2.set_baro_alt_noise(noise);
#endif
#if HAL_NAVEKF3_AVAILABLE
    EKF3.set_baro_alt_noise(noise);
#endif
}

// check if non-compass sensor is providing yaw.  Allows compass pre-arm checks to be bypassed
const EKFGSF_yaw *AP_AHRS::get_yaw_estimator(void) const
{
    switch (active_EKF_type()) {
#if HAL_NAVEKF2_AVAILABLE
    case EKFType::TWO:
        return EKF2.get_yawEstimator();
#endif
#if AP_AHRS_DCM_ENABLED
    case EKFType::DCM:
#if HAL_NAVEKF3_AVAILABLE
        return EKF3.get_yawEstimator();
#else
        return nullptr;
#endif
#endif
#if HAL_NAVEKF3_AVAILABLE
    case EKFType::THREE:
        return EKF3.get_yawEstimator();
#endif
#if AP_AHRS_SIM_ENABLED
    case EKFType::SIM:
#endif
#if AP_AHRS_EXTERNAL_ENABLED
    case EKFType::EXTERNAL:
#endif
        return nullptr;
    }
    // since there is no default case above, this is unreachable
    return nullptr;
}

// get current location estimate
bool AP_AHRS::get_location(Location &loc) const
{
    loc = state.location;
    return state.location_ok;
}

// return a wind estimation vector, in m/s; returns 0,0,0 on failure
bool AP_AHRS::wind_estimate(Vector3f &wind) const
{
    wind = state.wind_estimate;
    return state.wind_estimate_ok;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS::airspeed_estimate(float &airspeed_ret) const
{
    airspeed_ret = state.airspeed;
    return state.airspeed_ok;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS::airspeed_estimate(float &airspeed_ret, AP_AHRS::AirspeedEstimateType &type) const
{
    airspeed_ret = state.airspeed;
    type = state.airspeed_estimate_type;
    return state.airspeed_ok;
}

// return a true airspeed estimate (navigation airspeed) if
// available. return true if we have an estimate
bool AP_AHRS::airspeed_estimate_true(float &airspeed_ret) const
{
    airspeed_ret = state.airspeed_true;
    return state.airspeed_true_ok;
}

// return estimate of true airspeed vector in body frame in m/s
// returns false if estimate is unavailable
bool AP_AHRS::airspeed_vector_true(Vector3f &vec) const
{
    vec = state.airspeed_vec;
    return state.airspeed_vec_ok;
}

// return the quaternion defining the rotation from NED to XYZ (body) axes
bool AP_AHRS::get_quaternion(Quaternion &quat) const
{
    quat = state.quat;
    return state.quat_ok;
}

// returns the inertial navigation origin in lat/lon/alt
bool AP_AHRS::get_origin(Location &ret) const
{
    ret = state.origin;
    return state.origin_ok;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS::get_velocity_NED(Vector3f &vec) const
{
    vec = state.velocity_NED;
    return state.velocity_NED_ok;
}

// return location corresponding to vector relative to the
// vehicle's origin
bool AP_AHRS::get_location_from_origin_offset(Location &loc, const Vector3p &offset_ned) const
{
    if (!get_origin(loc)) {
        return false;
    }
    loc.offset(offset_ned);

    return true;
}

// return location corresponding to vector relative to the
// vehicle's home location
bool AP_AHRS::get_location_from_home_offset(Location &loc, const Vector3p &offset_ned) const
{
    if (!home_is_set()) {
        return false;
    }
    loc = get_home();
    loc.offset(offset_ned);

    return true;
}

// singleton instance
AP_AHRS *AP_AHRS::_singleton;

namespace AP {

AP_AHRS &ahrs()
{
    return *AP_AHRS::get_singleton();
}

}

#endif  // AP_AHRS_ENABLED
