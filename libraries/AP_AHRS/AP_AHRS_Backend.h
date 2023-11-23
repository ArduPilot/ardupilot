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
 *  AHRS (Attitude Heading Reference System) interface for ArduPilot
 *
 */

#include <AP_Math/AP_Math.h>
#include <inttypes.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Common/Location.h>

#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AP_AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter

enum class GPSUse : uint8_t {
    Disable = 0,
    Enable  = 1,
    EnableWithHeight = 2,
};

class AP_AHRS_Backend
{
public:

    // Constructor
    AP_AHRS_Backend() {}

    // empty virtual destructor
    virtual ~AP_AHRS_Backend() {}

    CLASS_NO_COPY(AP_AHRS_Backend);

    // structure to retrieve results from backends:
    struct Estimates {
        friend class AP_AHRS_DCM;
        friend class AP_AHRS_External;
        friend class AP_AHRS_SIM;

        bool initialised;
        bool healthy;

        uint8_t primary_imu_index;

        float roll_rad;
        float pitch_rad;
        float yaw_rad;
        Matrix3f dcm_matrix;
        Quaternion quat;

        bool attitude_valid;

        // return the quaternion defining the rotation from NED to XYZ
        // (body) axes
        bool get_quaternion(Quaternion &_quat) const WARN_IF_UNUSED {
            _quat = quat;
            return attitude_valid;
        }

        Vector3f gyro_estimate;
        Vector3f gyro_drift;
        Vector3f accel_ef;
        Vector3f accel_bias;

        bool get_velocity_NED(Vector3f &vel) const WARN_IF_UNUSED {
            if (velocity_NED_valid) {
                vel = velocity_NED;
            }
            return velocity_NED_valid;
        };
        bool get_vert_pos_rate_D(float &velocity) const WARN_IF_UNUSED {
            velocity = vert_pos_rate_D;
            return vert_pos_rate_D_valid;
        }

        // ground vector estimate in meters/second, in North/East order
        Vector2f groundspeed_vector;

        bool get_location(Location &loc) const WARN_IF_UNUSED {
            loc = location;
            return location_valid;
        };

        // origin-relative movement:
        bool get_origin(Location &ret) const {
            ret = origin;
            return origin_valid;
        }
        bool  get_relative_position_NED_origin(Vector3f &vec) const WARN_IF_UNUSED {
            vec = relative_position_NED_origin;
            return relative_position_NED_origin_valid;
        }

        bool  get_relative_position_NE_origin(Vector2f &vec) const WARN_IF_UNUSED {
            vec = relative_position_NE_origin;
            return relative_position_NE_origin_valid;
        }

        bool  get_relative_position_D_origin(float &down) const WARN_IF_UNUSED {
            down = relative_position_D_origin;
            return relative_position_D_origin_valid;
        }

        bool get_hagl(float &height) const WARN_IF_UNUSED {
            height = hagl;
            return hagl_valid;
        }

        bool wind_estimate(Vector3f &_wind) const WARN_IF_UNUSED {
            _wind = wind;
            return wind_valid;
        }

        void get_control_limits(float &_ekfGndSpdLimit, float &_controlScaleXY) const {
            _ekfGndSpdLimit = ekfGndSpdLimit;
            _controlScaleXY = controlScaleXY;
        }

    private:

        Vector3f velocity_NED;
        bool velocity_NED_valid;

        // A derivative of the vertical position in m/s which is
        // kinematically consistent with the vertical position is
        // required by some control loops.  This is different to the
        // vertical velocity from the EKF which is not always
        // consistent with the vertical position due to the various
        // errors that are being corrected for:
        float vert_pos_rate_D;
        bool vert_pos_rate_D_valid;

        Location location;
        bool location_valid;

        // origin for local position:
        Location origin;
        bool origin_valid;

        // position relative to origin in meters, North/East/Down
        // order. This will only be accurate if have_inertial_nav() is
        // true
        Vector3f relative_position_NED_origin;
        bool relative_position_NED_origin_valid;
        // a position relative to origin in meters, North/East
        // order
        Vector2f relative_position_NE_origin;
        bool relative_position_NE_origin_valid;
        float relative_position_D_origin;
        bool relative_position_D_origin_valid;

        float hagl;  // in metres
        bool hagl_valid;

        // wind estimate, earth frame, metres/second
        Vector3f wind;
        bool wind_valid;

        // control limits (with defaults):
        float ekfGndSpdLimit;
        float controlScaleXY;
    };

    // init sets up INS board orientation
    virtual void init();

    // Methods
    virtual void update() = 0;

    virtual void get_results(Estimates &results) = 0;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked
    virtual bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const = 0;

    // check all cores providing consistent attitudes for prearm checks
    virtual bool attitudes_consistent(char *failure_msg, const uint8_t failure_msg_len) const { return true; }

    // see if EKF lane switching is possible to avoid EKF failsafe
    virtual void check_lane_switch(void) {}

    // check if non-compass sensor is providing yaw.  Allows compass pre-arm checks to be bypassed
    virtual bool using_noncompass_for_yaw(void) const { return false; }

    // check if external nav is providing yaw
    virtual bool using_extnav_for_yaw(void) const { return false; }

    // request EKF yaw reset to try and avoid the need for an EKF lane switch or failsafe
    virtual void request_yaw_reset(void) {}

    // set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
    virtual void set_posvelyaw_source_set(uint8_t source_set_idx) {}

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    virtual void reset_gyro_drift(void) = 0;

    // reset the current attitude, used on new IMU calibration
    virtual void reset() = 0;

    // return an airspeed estimate if available. return true
    // if we have an estimate
    virtual bool airspeed_estimate(float &airspeed_ret) const WARN_IF_UNUSED { return false; }
    virtual bool airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const { return false; }

    // return a true airspeed estimate (navigation airspeed) if
    // available. return true if we have an estimate
    bool airspeed_estimate_true(float &airspeed_ret) const WARN_IF_UNUSED {
        if (!airspeed_estimate(airspeed_ret)) {
            return false;
        }
        airspeed_ret *= get_EAS2TAS();
        return true;
    }

    // return estimate of true airspeed vector in body frame in m/s
    // returns false if estimate is unavailable
    virtual bool airspeed_vector_true(Vector3f &vec) const WARN_IF_UNUSED {
        return false;
    }

    // get apparent to true airspeed ratio
    static float get_EAS2TAS(void);

    // return true if airspeed comes from an airspeed sensor, as
    // opposed to an IMU estimate
    static bool airspeed_sensor_enabled(void) {
    #if AP_AIRSPEED_ENABLED
        const AP_Airspeed *_airspeed = AP::airspeed();
        return _airspeed != nullptr && _airspeed->use() && _airspeed->healthy();
    #else
        return false;
    #endif
    }

    // return true if airspeed comes from a specific airspeed sensor, as
    // opposed to an IMU estimate
    static bool airspeed_sensor_enabled(uint8_t airspeed_index) {
    #if AP_AIRSPEED_ENABLED
        const AP_Airspeed *_airspeed = AP::airspeed();
        return _airspeed != nullptr && _airspeed->use(airspeed_index) && _airspeed->healthy(airspeed_index);
    #else
        return false;
    #endif
    }

    //
    virtual bool set_origin(const Location &loc) {
        return false;
    }

    // return true if we will use compass for yaw
    virtual bool use_compass(void) = 0;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    virtual uint32_t getLastYawResetAngle(float &yawAng) {
        return 0;
    };

    // return the amount of NE position change in metres due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosNorthEastReset(Vector2f &pos) WARN_IF_UNUSED {
        return 0;
    };

    // return the amount of NE velocity change in metres/sec due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastVelNorthEastReset(Vector2f &vel) const WARN_IF_UNUSED {
        return 0;
    };

    // return the amount of vertical position change due to the last reset in meters
    // returns the time of the last reset or 0 if no reset has ever occurred
    virtual uint32_t getLastPosDownReset(float &posDelta) WARN_IF_UNUSED {
        return 0;
    };

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    virtual bool resetHeightDatum(void) WARN_IF_UNUSED {
        return false;
    }

    // return the innovations for the specified instance
    // An out of range instance (eg -1) returns data for the primary instance
    virtual bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const {
        return false;
    }

    virtual bool get_filter_status(union nav_filter_status &status) const {
        return false;
    }

    // get_variances - provides the innovations normalised using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    virtual bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
        return false;
    }

    // get a source's velocity innovations.  source should be from 0 to 7 (see AP_NavEKF_Source::SourceXY)
    // returns true on success and results are placed in innovations and variances arguments
    virtual bool get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const WARN_IF_UNUSED {
        return false;
    }

    virtual void send_ekf_status_report(class GCS_MAVLINK &link) const = 0;

    // get_hgt_ctrl_limit - get maximum height to be observed by the
    // control loops in meters and a validity flag.  It will return
    // false when no limiting is required
    virtual bool get_hgt_ctrl_limit(float &limit) const WARN_IF_UNUSED { return false; };

    // Set to true if the terrain underneath is stable enough to be used as a height reference
    // this is not related to terrain following
    virtual void set_terrain_hgt_stable(bool stable) {}

};
