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
  support for serial connected AHRS systems
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ENABLED

#include "AP_ExternalAHRS.h"
#include "AP_ExternalAHRS_backend.h"
#include "AP_ExternalAHRS_VectorNav.h"
#include "AP_ExternalAHRS_MicroStrain5.h"
#include "AP_ExternalAHRS_MicroStrain7.h"
#include "AP_ExternalAHRS_InertialLabs.h"
#include "AP_ExternalAHRS_SBG.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS *AP_ExternalAHRS::_singleton;

// constructor
AP_ExternalAHRS::AP_ExternalAHRS()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
    if (rate.get() < 50) {
        // min 50Hz
        rate.set(50);
    }
}

#ifndef HAL_EXTERNAL_AHRS_DEFAULT
#define HAL_EXTERNAL_AHRS_DEFAULT 0
#endif


// table of user settable parameters
const AP_Param::GroupInfo AP_ExternalAHRS::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: AHRS type
    // @Description: Type of AHRS device
    // @Values: 0:None,1:VectorNav,2:MicroStrain5,5:InertialLabs,7:MicroStrain7,8:SBG
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_ExternalAHRS, devtype, HAL_EXTERNAL_AHRS_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: _RATE
    // @DisplayName: AHRS data rate
    // @Description: Requested rate for AHRS device
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_RATE", 2, AP_ExternalAHRS, rate, 50),

    // @Param: _OPTIONS
    // @DisplayName: External AHRS options
    // @Description: External AHRS options bitmask
    // @Bitmask: 0:Vector Nav use uncompensated values for accel gyro and mag.
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 3, AP_ExternalAHRS, options, 0),

    // @Param: _SENSORS
    // @DisplayName: External AHRS sensors
    // @Description: External AHRS sensors bitmask
    // @Bitmask: 0:GPS,1:IMU,2:Baro,3:Compass
    // @User: Advanced
    AP_GROUPINFO("_SENSORS", 4, AP_ExternalAHRS, sensors, 0xF),

    // @Param: _LOG_RATE
    // @DisplayName: AHRS logging rate
    // @Description: Logging rate for EARHS devices
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_LOG_RATE", 5, AP_ExternalAHRS, log_rate, 10),
    
    AP_GROUPEND
};


void AP_ExternalAHRS::init(void)
{
    if (rate.get() < 50) {
        // min 50Hz
        rate.set(50);
    }

    switch (DevType(devtype)) {
    case DevType::None:
        // nothing to do
        return;

#if AP_EXTERNAL_AHRS_VECTORNAV_ENABLED
    case DevType::VecNav:
        backend = NEW_NOTHROW AP_ExternalAHRS_VectorNav(this, state);
        return;
#endif

#if AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED
    case DevType::MicroStrain5:
        backend = NEW_NOTHROW AP_ExternalAHRS_MicroStrain5(this, state);
        return;
#endif

#if AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED
    case DevType::MicroStrain7:
        backend = NEW_NOTHROW AP_ExternalAHRS_MicroStrain7(this, state);
        return;
#endif

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
    case DevType::InertialLabs:
        backend = NEW_NOTHROW AP_ExternalAHRS_InertialLabs(this, state);
        return;
#endif

#if AP_EXTERNAL_AHRS_SBG_ENABLED
    case DevType::SBG:
        backend = NEW_NOTHROW AP_ExternalAHRS_SBG(this, state);
        return;
#endif // AP_EXTERNAL_AHRS_SBG_ENABLED

    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Unsupported ExternalAHRS type %u", unsigned(devtype));
}

bool AP_ExternalAHRS::enabled() const
{
    return DevType(devtype) != DevType::None;
}

// get serial port number for the uart, or -1 if not applicable
int8_t AP_ExternalAHRS::get_port(AvailableSensor sensor) const
{
    if (!backend || !has_sensor(sensor)) {
        return -1;
    }
    return backend->get_port();
};

// accessors for AP_AHRS
bool AP_ExternalAHRS::healthy(void) const
{
    return backend && backend->healthy();
}

bool AP_ExternalAHRS::initialised(void) const
{
    return backend && backend->initialised();
}

bool AP_ExternalAHRS::get_quaternion(Quaternion &quat)
{
    if (state.have_quaternion) {
        WITH_SEMAPHORE(state.sem);
        quat = state.quat;
        return true;
    }
    return false;
}

bool AP_ExternalAHRS::get_origin(Location &loc)
{
    if (state.have_origin) {
        WITH_SEMAPHORE(state.sem);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (!state.origin.initialised()) {
            AP_HAL::panic("Uninitialized origin in AP_ExternalAHRS.");
        }
#endif
        loc = state.origin;
        return true;
    }
    return false;
}

bool AP_ExternalAHRS::set_origin(const Location &loc)
{
    WITH_SEMAPHORE(state.sem);
    if (state.have_origin) {
        return false;
    }
    state.origin = loc;
    state.have_origin = true;
    return true;
}

bool AP_ExternalAHRS::get_location(Location &loc)
{
    if (!state.have_location) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    loc = state.location;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!loc.initialised()) {
        AP_HAL::panic("Uninitialized location in AP_ExternalAHRS.");
    }
#endif

    if (state.last_location_update_us != 0 &&
        state.have_velocity) {
        // extrapolate position based on velocity to cope with slow backends
        const float dt = (AP_HAL::micros() - state.last_location_update_us)*1.0e-6;
        if (dt < 1) {
            // only extrapolate for 1s max
            Vector3p ofs = state.velocity.topostype();
            ofs *= dt;
            loc.offset(ofs);
        }
    }

    return true;
}

Vector2f AP_ExternalAHRS::get_groundspeed_vector()
{
    WITH_SEMAPHORE(state.sem);
    Vector2f vec{state.velocity.x, state.velocity.y};
    return vec;
}

bool AP_ExternalAHRS::get_velocity_NED(Vector3f &vel)
{
    if (!state.have_velocity) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    vel = state.velocity;
    return true;
}

bool AP_ExternalAHRS::get_speed_down(float &speedD)
{
    if (!state.have_velocity) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    speedD = state.velocity.z;
    return true;
}

bool AP_ExternalAHRS::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (backend == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "ExternalAHRS: Invalid backend");
        return false;
    }
    if (!backend->pre_arm_check(failure_msg, failure_msg_len)) {
        return false;
    }
    // Verify the user has configured the GPS to accept EAHRS data.
    if (has_sensor(AvailableSensor::GPS)) {
        const auto eahrs_gps_sensors = backend->num_gps_sensors();

        const auto &gps = AP::gps();
        uint8_t n_configured_eahrs_gps = 0;
        for (uint8_t i = 0; i < GPS_MAX_INSTANCES; ++i) {
            const auto gps_type = gps.get_type(i);
            if (gps_type == AP_GPS::GPS_TYPE_EXTERNAL_AHRS) {
                n_configured_eahrs_gps++;
            }
        }

        // Once AP supports at least 3 GPS's, change to == and remove the second condition.
        // At that point, enforce that all GPS's in EAHRS can report to AP_GPS.
        if (n_configured_eahrs_gps < 1 && eahrs_gps_sensors >= 1) {
            hal.util->snprintf(failure_msg, failure_msg_len, "ExternalAHRS: Incorrect number of GPS sensors configured for EAHRS");
            return false;
        }
    }
    AP_AHRS &ahrs = AP::ahrs();
    if (ahrs.configured_ekf_type() == AP_AHRS::EKFType::EXTERNAL) {
        // when using EAHRS as the EKF source, we must have a valid position origin
        if (!state.have_origin) {
            hal.util->snprintf(failure_msg, failure_msg_len, "ExternalAHRS: No origin");
            return false;
        }
    }
    return true;
}

/*
  get filter status
 */
void AP_ExternalAHRS::get_filter_status(nav_filter_status &status) const
{
    status = {};
    if (backend) {
        backend->get_filter_status(status);
    }
}

/*
  get estimated variances, return false if not implemented
 */
bool AP_ExternalAHRS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    if (backend != nullptr) {
        return backend->get_variances(velVar, posVar, hgtVar, magVar, tasVar);
    }
    return false;
}

bool AP_ExternalAHRS::get_gyro(Vector3f &gyro)
{
    WITH_SEMAPHORE(state.sem);
    if (!has_sensor(AvailableSensor::IMU)) {
        return false;
    }
    gyro = state.gyro;
    return true;
}

bool AP_ExternalAHRS::get_accel(Vector3f &accel)
{
    WITH_SEMAPHORE(state.sem);
    if (!has_sensor(AvailableSensor::IMU)) {
        return false;
    }
    accel = state.accel;
    return true;
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS::send_status_report(GCS_MAVLINK &link) const
{
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;
    if (backend == nullptr || !backend->get_variances(velVar, posVar, hgtVar, magVar, tasVar)) {
        return;
    }

    uint16_t flags = 0;
    nav_filter_status filterStatus {};
    get_filter_status(filterStatus);

    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    const float mag_var = MAX(magVar.x, MAX(magVar.y, magVar.z));
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       velVar,
                                       posVar,
                                       hgtVar,
                                       mag_var, 0, 0);
}

void AP_ExternalAHRS::update(void)
{
    if (backend) {
        backend->update();
    }

    WITH_SEMAPHORE(state.sem);
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (enabled() && log_rate.get() > 0 && now_ms - last_log_ms >= uint32_t(1000U/log_rate.get())) {
        last_log_ms = now_ms;

        // @LoggerMessage: EAHR
        // @Description: External AHRS data
        // @Field: TimeUS: Time since system startup
        // @Field: Roll: euler roll
        // @Field: Pitch: euler pitch
        // @Field: Yaw: euler yaw
        // @Field: VN: velocity north
        // @Field: VE: velocity east
        // @Field: VD: velocity down
        // @Field: Lat: latitude
        // @Field: Lon: longitude
        // @Field: Alt: altitude AMSL
        // @Field: Flg: nav status flags

        float roll, pitch, yaw;
        state.quat.to_euler(roll, pitch, yaw);
        nav_filter_status filterStatus {};
        get_filter_status(filterStatus);

        AP::logger().WriteStreaming("EAHR", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lon,Alt,Flg",
                                    "sdddnnnDUm-",
                                    "F000000GG0-",
                                    "QffffffLLfI",
                                    AP_HAL::micros64(),
                                    degrees(roll), degrees(pitch), degrees(yaw),
                                    state.velocity.x, state.velocity.y, state.velocity.z,
                                    state.location.lat, state.location.lng, state.location.alt*0.01,
                                    filterStatus.value);

        // @LoggerMessage: EAHV
        // @Description: External AHRS variances
        // @Field: TimeUS: Time since system startup
        // @Field: Vel: velocity variance
        // @Field: Pos: position variance
        // @Field: Hgt: height variance
        // @Field: MagX: magnetic variance, X
        // @Field: MagY: magnetic variance, Y
        // @Field: MagZ: magnetic variance, Z
        // @Field: TAS: true airspeed variance

        float velVar, posVar, hgtVar, tasVar;
        Vector3f magVar;
        if (backend != nullptr && backend->get_variances(velVar, posVar, hgtVar, magVar, tasVar)) {
            AP::logger().WriteStreaming("EAHV", "TimeUS,Vel,Pos,Hgt,MagX,MagY,MagZ,TAS",
                                        "Qfffffff",
                                        AP_HAL::micros64(),
                                        velVar, posVar, hgtVar,
                                        magVar.x, magVar.y, magVar.z,
                                        tasVar);
        }
    }
#endif  // HAL_LOGGING_ENABLED
}

// Get model/type name
const char* AP_ExternalAHRS::get_name() const
{
    if (backend) {
        return backend->get_name();
    }
    return nullptr;
}

namespace AP {

AP_ExternalAHRS &externalAHRS()
{
    return *AP_ExternalAHRS::get_singleton();
}

};

#endif  // AP_EXTERNAL_AHRS_ENABLED

