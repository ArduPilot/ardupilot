/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  22 state EKF based on https://github.com/priseborough/InertialNav

  Converted from Matlab to C++ by Paul Riseborough

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
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Param/AP_Param.h>
#include "AP_Nav_Common.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

#include <AP_Math/vectorN.h>

// GPS pre-flight check bit locations
#define MASK_GPS_NSATS      (1<<0)
#define MASK_GPS_HDOP       (1<<1)
#define MASK_GPS_SPD_ERR    (1<<2)
#define MASK_GPS_POS_ERR    (1<<3)
#define MASK_GPS_YAW_ERR 	(1<<4)
#define MASK_GPS_POS_DRIFT  (1<<5)
#define MASK_GPS_VERT_SPD   (1<<6)
#define MASK_GPS_HORIZ_SPD  (1<<7)

class AP_AHRS;
class NavEKF_core;

class NavEKF
{
    friend class NavEKF_core;
    
public:
    // Constructor
    NavEKF(const AP_AHRS *ahrs, AP_Baro &baro, const RangeFinder &rng);

    // allow logging to determine if the EKF is enabled
    bool enabled(void) const {
        return (_enable != 0);
    }

    // This function is used to initialise the filter whilst moving, using the AHRS DCM solution
    // It should NOT be used to re-initialise after a timeout as DCM will also be corrupted
    bool InitialiseFilterDynamic(void);

    // Initialise the states from accelerometer and magnetometer data (if present)
    // This method can only be used when the vehicle is static
    bool InitialiseFilterBootstrap(void);

    // Update Filter States - this should be called whenever new IMU data is available
    void UpdateFilter(void);

    // Check basic filter health metrics and return a consolidated health status
    bool healthy(void) const;

    // Write the last calculated North East position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosNE(Vector2f &posNE) const;

    // Write the last calculated Down position relative to the reference point (m).
    // If a calculated solution is not available, use the best available data and return false
    // If false returned, do not use for flight control
    bool getPosD(float &posD) const;

    // return NED velocity in m/s
    void getVelNED(Vector3f &vel) const;

    // Return the rate of change of vertical position in the down diection (dPosD/dt) in m/s
    // This can be different to the z component of the EKF velocity state because it will fluctuate with height errors and corrections in the EKF
    // but will always be kinematically consistent with the z component of the EKF position state
    float getPosDownDerivative(void) const;

    // This returns the specific forces in the NED frame
    void getAccelNED(Vector3f &accelNED) const;

    // return body axis gyro bias estimates in rad/sec
    void getGyroBias(Vector3f &gyroBias) const;

    // reset body axis gyro bias estimates
    void resetGyroBias(void);

    // Resets the baro so that it reads zero at the current height
    // Resets the EKF height to zero
    // Adjusts the EKf origin height so that the EKF height + origin height is the same as before
    // Returns true if the height datum reset has been performed
    // If using a range finder for height no reset is performed and it returns false
    bool resetHeightDatum(void);

    // Commands the EKF to not use GPS.
    // This command must be sent prior to arming as it will only be actioned when the filter is in static mode
    // This command is forgotten by the EKF each time it goes back into static mode (eg the vehicle disarms)
    // Returns 0 if command rejected
    // Returns 1 if attitude, vertical velocity and vertical position will be provided
    // Returns 2 if attitude, 3D-velocity, vertical position and relative horizontal position will be provided
    uint8_t setInhibitGPS(void);

    // return the horizontal speed limit in m/s set by optical flow sensor limits
    // return the scale factor to be applied to navigation velocity gains to compensate for increase in velocity noise with height when using optical flow
    void getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const;

    // return weighting of first IMU in blending function
    void getIMU1Weighting(float &ret) const;

    // return the individual Z-accel bias estimates in m/s^2
    void getAccelZBias(float &zbias1, float &zbias2) const;

    // return the NED wind speed estimates in m/s (positive is air moving in the direction of the axis)
    void getWind(Vector3f &wind) const;

    // return earth magnetic field estimates in measurement units / 1000
    void getMagNED(Vector3f &magNED) const;

    // return body magnetic field estimates in measurement units / 1000
    void getMagXYZ(Vector3f &magXYZ) const;

    // Return estimated magnetometer offsets
    // Return true if magnetometer offsets are valid
    bool getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const;

    // Return the last calculated latitude, longitude and height in WGS-84
    // If a calculated location isn't available, return a raw GPS measurement
    // The status will return true if a calculation or raw measurement is available
    // The getFilterStatus() function provides a more detailed description of data health and must be checked if data is to be used for flight control
    bool getLLH(struct Location &loc) const;

    // return the latitude and longitude and height used to set the NED origin
    // All NED positions calculated by the filter are relative to this location
    // Returns false if the origin has not been set
    bool getOriginLLH(struct Location &loc) const;

    // set the latitude and longitude and height used to set the NED origin
    // All NED positions calcualted by the filter will be relative to this location
    // The origin cannot be set if the filter is in a flight mode (eg vehicle armed)
    // Returns false if the filter has rejected the attempt to set the origin
    bool setOriginLLH(struct Location &loc);

    // return estimated height above ground level
    // return false if ground height is not being estimated.
    bool getHAGL(float &HAGL) const;

    // return the Euler roll, pitch and yaw angle in radians
    void getEulerAngles(Vector3f &eulers) const;

    // return the transformation matrix from XYZ (body) to NED axes
    void getRotationBodyToNED(Matrix3f &mat) const;

    // return the quaternions defining the rotation from NED to XYZ (body) axes
    void getQuaternion(Quaternion &quat) const;

    // return the innovations for the NED Pos, NED Vel, XYZ Mag and Vtas measurements
    void  getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov) const;

    // return the innovation consistency test ratios for the velocity, position, magnetometer and true airspeed measurements
    void  getVariances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset) const;

    // should we use the compass? This is public so it can be used for
    // reporting via ahrs.use_compass()
    bool use_compass(void) const;

    // write the raw optical flow measurements
    // rawFlowQuality is a measured of quality between 0 and 255, with 255 being the best quality
    // rawFlowRates are the optical flow rates in rad/sec about the X and Y sensor axes.
    // rawGyroRates are the sensor rotation rates in rad/sec measured by the sensors internal gyro
    // The sign convention is that a RH physical rotation of the sensor about an axis produces both a positive flow and gyro rate
    // msecFlowMeas is the scheduler time in msec when the optical flow data was received from the sensor.
    void  writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas);

    // return data for debugging optical flow fusion
    void getFlowDebug(float &varFlow, float &gndOffset, float &flowInnovX, float &flowInnovY, float &auxInnov, float &HAGL, float &rngInnov, float &range, float &gndOffsetErr) const;

    // called by vehicle code to specify that a takeoff is happening
    // causes the EKF to compensate for expected barometer errors due to ground effect
    void setTakeoffExpected(bool val);

    // called by vehicle code to specify that a touchdown is expected to happen
    // causes the EKF to compensate for expected barometer errors due to ground effect
    void setTouchdownExpected(bool val);

    /*
    return the filter fault status as a bitmasked integer
     0 = quaternions are NaN
     1 = velocities are NaN
     2 = badly conditioned X magnetometer fusion
     3 = badly conditioned Y magnetometer fusion
     4 = badly conditioned Z magnetometer fusion
     5 = badly conditioned airspeed fusion
     6 = badly conditioned synthetic sideslip fusion
     7 = filter is not initialised
    */
    void  getFilterFaults(uint16_t &faults) const;

    /*
    return filter timeout status as a bitmasked integer
     0 = position measurement timeout
     1 = velocity measurement timeout
     2 = height measurement timeout
     3 = magnetometer measurement timeout
     4 = true airspeed measurement timeout
     5 = unassigned
     6 = unassigned
     7 = unassigned
    */
    void  getFilterTimeouts(uint8_t &timeouts) const;

    /*
    return filter status flags
    */
    void  getFilterStatus(nav_filter_status &status) const;

    /*
    return filter gps quality check status
    */
    void  getFilterGpsStatus(nav_gps_status &status) const;

    // send an EKF_STATUS_REPORT message to GCS
    void send_status_report(mavlink_channel_t chan);

    // provides the height limit to be observed by the control loops
    // returns false if no height limiting is required
    // this is needed to ensure the vehicle does not fly too high when using optical flow navigation
    bool getHeightControlLimit(float &height) const;

    // returns true of the EKF thinks the GPS is glitching
    bool getGpsGlitchStatus(void) const;

    // return the amount of yaw angle change due to the last yaw angle reset in radians
    // returns the time of the last yaw angle reset or 0 if no reset has ever occurred
    uint32_t getLastYawResetAngle(float &yawAng) const;

    // return the amount of NE position change due to the last position reset in metres
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos) const;

    // return the amount of NE velocity change due to the last velocity reset in metres/sec
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastVelNorthEastReset(Vector2f &vel) const;

    // report any reason for why the backend is refusing to initialise
    const char *prearm_failure_reason(void) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    const AP_AHRS *_ahrs;
    AP_Baro &_baro;
    const RangeFinder &_rng;
    NavEKF_core *core;

    // EKF Mavlink Tuneable Parameters
    AP_Int8  _enable;               // zero to disable EKF1
    AP_Float _gpsHorizVelNoise;     // GPS horizontal velocity measurement noise : m/s
    AP_Float _gpsVertVelNoise;      // GPS vertical velocity measurement noise : m/s
    AP_Float _gpsHorizPosNoise;     // GPS horizontal position measurement noise m
    AP_Float _baroAltNoise;         // Baro height measurement noise : m^2
    AP_Float _magNoise;             // magnetometer measurement noise : gauss
    AP_Float _easNoise;             // equivalent airspeed measurement noise : m/s
    AP_Float _windVelProcessNoise;  // wind velocity state process noise : m/s^2
    AP_Float _wndVarHgtRateScale;   // scale factor applied to wind process noise due to height rate
    AP_Float _magEarthProcessNoise; // earth magnetic field process noise : gauss/sec
    AP_Float _magBodyProcessNoise;  // earth magnetic field process noise : gauss/sec
    AP_Float _gyrNoise;             // gyro process noise : rad/s
    AP_Float _accNoise;             // accelerometer process noise : m/s^2
    AP_Float _gyroBiasProcessNoise; // gyro bias state process noise : rad/s
    AP_Float _accelBiasProcessNoise;// accel bias state process noise : m/s^2
    AP_Int16 _msecVelDelay;         // effective average delay of GPS velocity measurements rel to IMU (msec)
    AP_Int16 _msecPosDelay;         // effective average delay of GPS position measurements rel to (msec)
    AP_Int8  _fusionModeGPS;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity
    AP_Int8  _gpsVelInnovGate;      // Number of standard deviations applied to GPS velocity innovation consistency check
    AP_Int8  _gpsPosInnovGate;      // Number of standard deviations applied to GPS position innovation consistency check
    AP_Int8  _hgtInnovGate;         // Number of standard deviations applied to height innovation consistency check
    AP_Int8  _magInnovGate;         // Number of standard deviations applied to magnetometer innovation consistency check
    AP_Int8  _tasInnovGate;         // Number of standard deviations applied to true airspeed innovation consistency check
    AP_Int8  _magCal;               // Sets activation condition for in-flight magnetometer calibration
    AP_Int16 _gpsGlitchAccelMax;    // Maximum allowed discrepancy between inertial and GPS Horizontal acceleration before GPS data is ignored : cm/s^2
    AP_Int8 _gpsGlitchRadiusMax;    // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    AP_Int8 _gndGradientSigma;      // RMS terrain gradient percentage assumed by the terrain height estimation.
    AP_Float _flowNoise;            // optical flow rate measurement noise
    AP_Int8  _flowInnovGate;        // Number of standard deviations applied to optical flow innovation consistency check
    AP_Int8  _msecFLowDelay;        // effective average delay of optical flow measurements rel to IMU (msec)
    AP_Int8  _rngInnovGate;         // Number of standard deviations applied to range finder innovation consistency check
    AP_Float _maxFlowRate;          // Maximum flow rate magnitude that will be accepted by the filter
    AP_Int8 _fallback;              // EKF-to-DCM fallback strictness. 0 = trust EKF more, 1 = fallback more conservatively.
    AP_Int8 _altSource;             // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder.
    AP_Int8 _gpsCheck;              // Bitmask controlling which preflight GPS checks are bypassed

};
