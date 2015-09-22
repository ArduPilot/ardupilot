/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  24 state EKF based on https://github.com/priseborough/InertialNav
  Converted from Matlab to C++ by Paul Riseborough

  EKF Tuning parameters refactored by Tom Cauchois

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

#ifndef AP_NavEKF2_Tuning
#define AP_NavEKF2_Tuning

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class NavEKF2
{
public:
    static const struct AP_Param::GroupInfo var_info[];
    NavEKF2();

    // EKF Mavlink Tuneable Parameters
    AP_Int8  _enable;               // zero to disable EKF2
    AP_Float _gpsHorizVelNoise;     // GPS horizontal velocity measurement noise : m/s
    AP_Float _gpsVertVelNoise;      // GPS vertical velocity measurement noise : m/s
    AP_Float _gpsHorizPosNoise;     // GPS horizontal position measurement noise m
    AP_Float _baroAltNoise;         // Baro height measurement noise : m^2
    AP_Float _magNoise;             // magnetometer measurement noise : gauss
    AP_Float _easNoise;             // equivalent airspeed measurement noise : m/s
    AP_Float _windVelProcessNoise;  // wind velocity state process noise : m/s^2
    AP_Float _wndVarHgtRateScale;   // scale factor applied to wind process noise due to height rate
    AP_Float _magProcessNoise;      // magnetic field process noise : gauss/sec
    AP_Float _gyrNoise;             // gyro process noise : rad/s
    AP_Float _accNoise;             // accelerometer process noise : m/s^2
    AP_Float _gyroBiasProcessNoise; // gyro bias state process noise : rad/s
    AP_Float _accelBiasProcessNoise;// accel bias state process noise : m/s^2
    AP_Int16 _msecGpsDelay;         // effective average delay of GPS measurements relative to time of receipt (msec)
    AP_Int8  _fusionModeGPS;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity
    AP_Int8  _gpsVelInnovGate;      // Number of standard deviations applied to GPS velocity innovation consistency check
    AP_Int8  _gpsPosInnovGate;      // Number of standard deviations applied to GPS position innovation consistency check
    AP_Int8  _hgtInnovGate;         // Number of standard deviations applied to height innovation consistency check
    AP_Int8  _magInnovGate;         // Number of standard deviations applied to magnetometer innovation consistency check
    AP_Int8  _tasInnovGate;         // Number of standard deviations applied to true airspeed innovation consistency check
    AP_Int8  _magCal;               // Sets activation condition for in-flight magnetometer calibration
    AP_Int8 _gpsGlitchRadiusMax;    // Maximum allowed discrepancy between inertial and GPS Horizontal position before GPS glitch is declared : m
    AP_Int8 _gndGradientSigma;      // RMS terrain gradient percentage assumed by the terrain height estimation.
    AP_Float _flowNoise;            // optical flow rate measurement noise
    AP_Int8  _flowInnovGate;        // Number of standard deviations applied to optical flow innovation consistency check
    AP_Int8  _msecFlowDelay;        // effective average delay of optical flow measurements rel to IMU (msec)
    AP_Int8  _rngInnovGate;         // Number of standard deviations applied to range finder innovation consistency check
    AP_Float _maxFlowRate;          // Maximum flow rate magnitude that will be accepted by the filter
    AP_Int8 _fallback;              // EKF-to-DCM fallback strictness. 0 = trust EKF more, 1 = fallback more conservatively.
    AP_Int8 _altSource;             // Primary alt source during optical flow navigation. 0 = use Baro, 1 = use range finder.
    AP_Float _gyroScaleProcessNoise;// gyro scale factor state process noise : 1/s

    // Tuning parameters
    const float gpsNEVelVarAccScale;    // Scale factor applied to NE velocity measurement variance due to manoeuvre acceleration
    const float gpsDVelVarAccScale;     // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration
    const float gpsPosVarAccScale;      // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    const uint16_t msecHgtDelay;        // Height measurement delay (msec)
    const uint16_t msecMagDelay;        // Magnetometer measurement delay (msec)
    const uint16_t msecTasDelay;        // Airspeed measurement delay (msec)
    const uint16_t gpsRetryTimeUseTAS;  // GPS retry time with airspeed measurements (msec)
    const uint16_t gpsRetryTimeNoTAS;   // GPS retry time without airspeed measurements (msec)
    const uint16_t gpsFailTimeWithFlow; // If we have no GPs for longer than this and we have optical flow, then we will switch across to using optical flow (msec)
    const uint16_t hgtRetryTimeMode0;   // Height retry time with vertical velocity measurement (msec)
    const uint16_t hgtRetryTimeMode12;  // Height retry time without vertical velocity measurement (msec)
    const uint16_t tasRetryTime;        // True airspeed timeout and retry interval (msec)
    const uint32_t magFailTimeLimit_ms; // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    const float magVarRateScale;        // scale factor applied to magnetometer variance due to angular rate
    const float gyroBiasNoiseScaler;    // scale factor applied to gyro bias state process noise when on ground
    const float accelBiasNoiseScaler;   // scale factor applied to accel bias state process noise when on ground
    const uint16_t msecGpsAvg;          // average number of msec between GPS measurements
    const uint16_t msecHgtAvg;          // average number of msec between height measurements
    const uint16_t msecMagAvg;          // average number of msec between magnetometer measurements
    const uint16_t msecBetaAvg;         // average number of msec between synthetic sideslip measurements
    const uint16_t msecBetaMax;         // maximum number of msec between synthetic sideslip measurements
    const uint16_t msecFlowAvg;         // average number of msec between optical flow measurements
    const float dtVelPos;               // number of seconds between position and velocity corrections. This should be a multiple of the imu update interval.
    const float covTimeStepMax;         // maximum time (sec) between covariance prediction updates
    const float covDelAngMax;           // maximum delta angle between covariance prediction updates
    const uint32_t TASmsecMax;          // maximum allowed interval between airspeed measurement updates
    const float DCM33FlowMin;           // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    const float fScaleFactorPnoise;     // Process noise added to focal length scale factor state variance at each time step
    const uint8_t flowTimeDeltaAvg_ms;  // average interval between optical flow measurements (msec)
    const uint32_t flowIntervalMax_ms;  // maximum allowable time between flow fusion events
    const uint16_t gndEffectTimeout_ms;      // time in msec that ground effect mode is active after being activated
    const float gndEffectBaroScaler;    // scaler applied to the barometer observation variance when ground effect mode is active
};

#endif //AP_NavEKF2
