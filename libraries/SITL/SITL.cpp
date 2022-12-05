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
    SITL.cpp - software in the loop state
*/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "SITL.h"

#if AP_SIM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#ifdef SFML_JOYSTICK
  #ifdef HAVE_SFML_GRAPHICS_HPP
    #include <SFML/Window/Joystick.hpp>
  #elif HAVE_SFML_GRAPHIC_H
    #include <SFML/Window/Joystick.h>
  #endif
#endif // SFML_JOYSTICK

extern const AP_HAL::HAL& hal;

namespace SITL {

SIM *SIM::_singleton = nullptr;

// table of user settable parameters
const AP_Param::GroupInfo SIM::var_info[] = {
    
    AP_GROUPINFO("DRIFT_SPEED",    5, SIM,  drift_speed, 0.05f),
    AP_GROUPINFO("DRIFT_TIME",     6, SIM,  drift_time,  5),
    AP_GROUPINFO("ENGINE_MUL",     8, SIM,  engine_mul,  1),
    AP_GROUPINFO("WIND_SPD",       9, SIM,  wind_speed,  0),
    AP_GROUPINFO("WIND_DIR",      10, SIM,  wind_direction,  180),
    AP_GROUPINFO("WIND_TURB",     11, SIM,  wind_turbulance,  0),
    AP_GROUPINFO("SERVO_SPEED",   16, SIM,  servo_speed,  0.14),
    AP_GROUPINFO("SONAR_ROT",     17, SIM,  sonar_rot, Rotation::ROTATION_PITCH_270),
    AP_GROUPINFO("BATT_VOLTAGE",  19, SIM,  batt_voltage,  12.6f),
    AP_GROUPINFO("BATT_CAP_AH",   20, SIM,  batt_capacity_ah,  0),
    AP_GROUPINFO("SONAR_GLITCH",  23, SIM,  sonar_glitch, 0),
    AP_GROUPINFO("SONAR_RND",     24, SIM,  sonar_noise, 0),
    AP_GROUPINFO("RC_FAIL",       25, SIM,  rc_fail, 0),
    AP_GROUPINFO("FLOAT_EXCEPT",  28, SIM,  float_exception, 1),
    AP_GROUPINFO("SONAR_SCALE",   32, SIM,  sonar_scale, 12.1212f),
    AP_GROUPINFO("FLOW_ENABLE",   33, SIM,  flow_enable, 0),
    AP_GROUPINFO("TERRAIN",       34, SIM,  terrain_enable, 1),
    AP_GROUPINFO("FLOW_RATE",     35, SIM,  flow_rate, 10),
    AP_GROUPINFO("FLOW_DELAY",    36, SIM,  flow_delay, 0),
    AP_GROUPINFO("ADSB_COUNT",    45, SIM,  adsb_plane_count, -1),
    AP_GROUPINFO("ADSB_RADIUS",   46, SIM,  adsb_radius_m, 10000),
    AP_GROUPINFO("ADSB_ALT",      47, SIM,  adsb_altitude_m, 1000),
    AP_GROUPINFO("PIN_MASK",      50, SIM,  pin_mask, 0),
    AP_GROUPINFO("ADSB_TX",       51, SIM,  adsb_tx, 0),
    AP_GROUPINFO("SPEEDUP",       52, SIM,  speedup, -1),
    AP_GROUPINFO("IMU_POS",       53, SIM,  imu_pos_offset, 0),
    AP_SUBGROUPEXTENSION("",      54, SIM,  var_ins),
    AP_GROUPINFO("SONAR_POS",     55, SIM,  rngfnd_pos_offset, 0),
    AP_GROUPINFO("FLOW_POS",      56, SIM,  optflow_pos_offset, 0),
    AP_GROUPINFO("ENGINE_FAIL",   58, SIM,  engine_fail,  0),
#if AP_SIM_SHIP_ENABLED
    AP_SUBGROUPINFO(shipsim, "SHIP_", 59, SIM, ShipSim),
#endif
    AP_SUBGROUPEXTENSION("",      60, SIM,  var_mag),
#if HAL_SIM_GPS_ENABLED
    AP_SUBGROUPEXTENSION("",      61, SIM,  var_gps),
#endif
    AP_SUBGROUPEXTENSION("",      62, SIM,  var_info3),
    AP_SUBGROUPEXTENSION("",      63, SIM,  var_info2),
    AP_GROUPEND
};

// second table of user settable parameters for SITL. 
const AP_Param::GroupInfo SIM::var_info2[] = {
    AP_GROUPINFO("TEMP_START",   1, SIM,  temp_start,  25),
    AP_GROUPINFO("TEMP_BRD_OFF", 2, SIM,  temp_board_offset, 20),
    AP_GROUPINFO("TEMP_TCONST",  3, SIM,  temp_tconst, 30),
    AP_GROUPINFO("TEMP_BFACTOR", 4, SIM,  temp_baro_factor, 0),

    AP_GROUPINFO("WIND_DIR_Z",  10, SIM,  wind_dir_z,     0),
    AP_GROUPINFO("WIND_T"      ,15, SIM,  wind_type, SIM::WIND_TYPE_SQRT),
    AP_GROUPINFO("WIND_T_ALT"  ,16, SIM,  wind_type_alt, 60),
    AP_GROUPINFO("WIND_T_COEF", 17, SIM,  wind_type_coef, 0.01f),
    AP_GROUPINFO("RC_CHANCOUNT",21, SIM,  rc_chancount, 16),
    // @Group: SPR_
    // @Path: ./SIM_Sprayer.cpp
    AP_SUBGROUPINFO(sprayer_sim, "SPR_", 22, SIM, Sprayer),
    // @Group: GRPS_
    // @Path: ./SIM_Gripper_Servo.cpp
    AP_SUBGROUPINFO(gripper_sim, "GRPS_", 23, SIM, Gripper_Servo),
    // @Group: GRPE_
    // @Path: ./SIM_Gripper_EPM.cpp
    AP_SUBGROUPINFO(gripper_epm_sim, "GRPE_", 24, SIM, Gripper_EPM),

    // weight on wheels pin
    AP_GROUPINFO("WOW_PIN",     25, SIM,  wow_pin, -1),

    // vibration frequencies on each axis
    AP_GROUPINFO("VIB_FREQ",   26, SIM,  vibe_freq, 0),

    // @Path: ./SIM_Parachute.cpp
    AP_SUBGROUPINFO(parachute_sim, "PARA_", 27, SIM, Parachute),

    // enable bandwidth limitting on telemetry ports:
    AP_GROUPINFO("BAUDLIMIT_EN",   28, SIM,  telem_baudlimit_enable, 0),

    // @Group: PLD_
    // @Path: ./SIM_Precland.cpp
    AP_SUBGROUPINFO(precland_sim, "PLD_", 29, SIM, SIM_Precland),

    // apply a force to the vehicle over a period of time:
    AP_GROUPINFO("SHOVE_X",     30, SIM,  shove.x, 0),
    AP_GROUPINFO("SHOVE_Y",     31, SIM,  shove.y, 0),
    AP_GROUPINFO("SHOVE_Z",     32, SIM,  shove.z, 0),
    AP_GROUPINFO("SHOVE_TIME",  33, SIM,  shove.t, 0),
    
    // optical flow sensor measurement noise in rad/sec
    AP_GROUPINFO("FLOW_RND",   34, SIM,  flow_noise,  0.05f),

    AP_GROUPINFO("TWIST_X",     37, SIM,  twist.x, 0),
    AP_GROUPINFO("TWIST_Y",     38, SIM,  twist.y, 0),
    AP_GROUPINFO("TWIST_Z",     39, SIM,  twist.z, 0),
    AP_GROUPINFO("TWIST_TIME",  40, SIM,  twist.t, 0),

    AP_GROUPINFO("GND_BEHAV",   41, SIM,  gnd_behav, -1),

    // sailboat wave and tide simulation parameters
    AP_GROUPINFO("WAVE_ENABLE", 44, SIM,  wave.enable, 0.0f),
    AP_GROUPINFO("WAVE_LENGTH", 45, SIM,  wave.length, 10.0f),
    AP_GROUPINFO("WAVE_AMP",    46, SIM,  wave.amp, 0.5f),
    AP_GROUPINFO("WAVE_DIR",    47, SIM,  wave.direction, 0.0f),
    AP_GROUPINFO("WAVE_SPEED",  48, SIM,  wave.speed, 0.5f),
    AP_GROUPINFO("TIDE_DIR",    49, SIM,  tide.direction, 0.0f),
    AP_GROUPINFO("TIDE_SPEED",  50, SIM,  tide.speed, 0.0f),

    // the following coordinates are for CMAC, in Canberra
    AP_GROUPINFO("OPOS_LAT",    51, SIM,  opos.lat, -35.363261f),
    AP_GROUPINFO("OPOS_LNG",    52, SIM,  opos.lng, 149.165230f),
    AP_GROUPINFO("OPOS_ALT",    53, SIM,  opos.alt, 584.0f),
    AP_GROUPINFO("OPOS_HDG",    54, SIM,  opos.hdg, 353.0f),

    // extra delay per main loop
    AP_GROUPINFO("LOOP_DELAY",  55, SIM,  loop_delay, 0),

    // @Path: ./SIM_Buzzer.cpp
    AP_SUBGROUPINFO(buzzer_sim, "BZ_", 56, SIM, Buzzer),

    // @Path: ./SIM_ToneAlarm.cpp
    AP_SUBGROUPINFO(tonealarm_sim, "TA_", 57, SIM, ToneAlarm),

    AP_GROUPINFO("EFI_TYPE",    58, SIM,  efi_type,  SIM::EFI_TYPE_NONE),

    AP_GROUPINFO("SAFETY_STATE",    59, SIM,  _safety_switch_state, 0),

    // motor harmonics
    AP_GROUPINFO("VIB_MOT_HMNC", 60, SIM,  vibe_motor_harmonics, 1),

    // motor mask, allowing external simulators to mark motors
    AP_GROUPINFO("VIB_MOT_MASK", 5, SIM,  vibe_motor_mask, 0),
    
    // max motor vibration frequency
    AP_GROUPINFO("VIB_MOT_MAX", 61, SIM,  vibe_motor, 0.0f),
    // minimum throttle for simulated ins noise
    AP_GROUPINFO("INS_THR_MIN", 62, SIM,  ins_noise_throttle_min, 0.1f),
    // amplitude scaling of motor noise relative to gyro/accel noise
    AP_GROUPINFO("VIB_MOT_MULT", 63, SIM,  vibe_motor_scale, 1.0f),


    AP_GROUPEND

};

// third table of user settable parameters for SITL. 
const AP_Param::GroupInfo SIM::var_info3[] = {
    AP_GROUPINFO("ODOM_ENABLE",   1, SIM,  odom_enable, 0),

    AP_GROUPINFO("LED_LAYOUT",    11, SIM, led_layout, 0),

    // Scenario for thermalling simulation, for soaring
    AP_GROUPINFO("THML_SCENARI",  12, SIM,  thermal_scenario, 0),

    // vicon sensor position (position offsets in body frame)
    AP_GROUPINFO("VICON_POS",     14, SIM,  vicon_pos_offset, 0),

    // Buyoancy for submarines
    AP_GROUPINFO_FRAME("BUOYANCY", 15, SIM, buoyancy, 1, AP_PARAM_FRAME_SUB),

    // vicon glitch in NED frame
    AP_GROUPINFO("VICON_GLIT",    16, SIM,  vicon_glitch, 0),

    // vicon failure
    AP_GROUPINFO("VICON_FAIL",    17, SIM,  vicon_fail, 0),

    // vicon yaw (in earth frame)
    AP_GROUPINFO("VICON_YAW",     18, SIM,  vicon_yaw, 0),

    // vicon yaw error in degrees (added to reported yaw sent to vehicle)
    AP_GROUPINFO("VICON_YAWERR",  19, SIM,  vicon_yaw_error, 0),

    // vicon message type mask
    AP_GROUPINFO("VICON_TMASK",   20, SIM,  vicon_type_mask, 3),

    // vicon velocity glitch in NED frame
    AP_GROUPINFO("VICON_VGLI",    21, SIM,  vicon_vel_glitch, 0),

    AP_GROUPINFO("RATE_HZ",  22, SIM,  loop_rate_hz, 1200),

    // count of simulated IMUs
    AP_GROUPINFO("IMU_COUNT",    23, SIM,  imu_count,  2),

    // @Path: ./SIM_FETtecOneWireESC.cpp
    AP_SUBGROUPINFO(fetteconewireesc_sim, "FTOWESC_", 30, SIM, FETtecOneWireESC),

    // @Path: ./SIM_RichenPower.cpp
    AP_SUBGROUPINFO(richenpower_sim, "RICH_", 31, SIM, RichenPower),

    // @Path: ./SIM_IntelligentEnergy24.cpp
    AP_SUBGROUPINFO(ie24_sim, "IE24_", 32, SIM, IntelligentEnergy24),

    // user settable barometer parameters
    AP_GROUPINFO("BARO_COUNT",    33, SIM,  baro_count, 2),

    AP_SUBGROUPINFO(baro[0], "BARO_", 34, SIM, SIM::BaroParm),
#if BARO_MAX_INSTANCES > 1
    AP_SUBGROUPINFO(baro[1], "BAR2_", 35, SIM, SIM::BaroParm),
#endif
#if BARO_MAX_INSTANCES > 2
    AP_SUBGROUPINFO(baro[2], "BAR3_", 36, SIM, SIM::BaroParm),
#endif

    AP_GROUPINFO("TIME_JITTER",  37, SIM,  loop_time_jitter_us, 0),

    // user settable parameters for the 1st barometer
    // @Param: BARO_RND
    // @DisplayName: Baro Noise
    // @Description: Amount of (evenly-distributed) noise injected into the 1st baro
    // @Units: m
    // @User: Advanced

    // @Param: BARO_GLITCH
    // @DisplayName: Baro Glitch
    // @Description: Glitch for 1st baro
    // @Units: m
    // @User: Advanced

    // user settable parameters for the 2nd barometer
    // @Param: BAR2_RND
    // @DisplayName: Baro2 Noise
    // @Description: Amount of (evenly-distributed) noise injected into the 2nd baro
    // @Units: m
    // @User: Advanced

    // @Param: BAR2_GLITCH
    // @DisplayName: Baro2 Glitch
    // @Description: Glitch for 2nd baro
    // @Units: m
    // @User: Advanced

    // user settable parameters for the 3rd barometer
    // @Param: BAR3_RND
    // @DisplayName: Baro3 Noise
    // @Description: Amount of (evenly-distributed) noise injected into the 3rd baro
    // @Units: m
    // @User: Advanced

    // @Param: BAR3_GLITCH
    // @DisplayName: Baro3 Glitch
    // @Description: Glitch for 2nd baro
    // @Units: m
    // @User: Advanced

    AP_GROUPINFO("ESC_TELEM", 40, SIM, esc_telem, 1),

    AP_GROUPINFO("ESC_ARM_RPM", 41, SIM,  esc_rpm_armed, 0.0f),

    AP_SUBGROUPINFO(airspeed[0], "ARSPD_", 50, SIM, SIM::AirspeedParm),
#if AIRSPEED_MAX_SENSORS > 1
    AP_SUBGROUPINFO(airspeed[1], "ARSPD2_", 51, SIM, SIM::AirspeedParm),
#endif


#ifdef SFML_JOYSTICK
    AP_SUBGROUPEXTENSION("",      63, SIM,  var_sfml_joystick),
#endif // SFML_JOYSTICK

    AP_GROUPEND
};

// user settable parameters for the barometers
const AP_Param::GroupInfo SIM::BaroParm::var_info[] = {
    AP_GROUPINFO("RND",      1, SIM::BaroParm,  noise, 0.2f),
    AP_GROUPINFO("DRIFT",    2, SIM::BaroParm,  drift, 0),
    AP_GROUPINFO("DISABLE",  3, SIM::BaroParm,  disable, 0),
    AP_GROUPINFO("GLITCH",   4, SIM::BaroParm,  glitch, 0),
    AP_GROUPINFO("FREEZE",   5, SIM::BaroParm,  freeze, 0),
    AP_GROUPINFO("DELAY",    6, SIM::BaroParm,  delay, 0),

    // wind coeffients
    AP_GROUPINFO("WCF_FWD", 7,  SIM::BaroParm, wcof_xp, 0.0),
    AP_GROUPINFO("WCF_BAK", 8,  SIM::BaroParm, wcof_xn, 0.0),
    AP_GROUPINFO("WCF_RGT", 9,  SIM::BaroParm, wcof_yp, 0.0),
    AP_GROUPINFO("WCF_LFT", 10, SIM::BaroParm, wcof_yn, 0.0),
    AP_GROUPEND
};

// user settable parameters for airspeed sensors
const AP_Param::GroupInfo SIM::AirspeedParm::var_info[] = {
        // user settable parameters for the 1st airspeed sensor
    AP_GROUPINFO("RND",     1, SIM::AirspeedParm,  noise, 2.0),
    AP_GROUPINFO("OFS",     2, SIM::AirspeedParm,  offset, 2013),
    AP_GROUPINFO("FAIL",    3, SIM::AirspeedParm,  fail, 0),
    AP_GROUPINFO("FAILP",   4, SIM::AirspeedParm,  fail_pressure, 0),
    AP_GROUPINFO("PITOT",   5, SIM::AirspeedParm,  fail_pitot_pressure, 0),
    AP_GROUPINFO("SIGN",    6, SIM::AirspeedParm,  signflip, 0),
    AP_GROUPINFO("RATIO",   7, SIM::AirspeedParm,  ratio, 1.99),
    AP_GROUPEND
};

#if HAL_SIM_GPS_ENABLED
// GPS SITL parameters
const AP_Param::GroupInfo SIM::var_gps[] = {
    AP_GROUPINFO("GPS_DISABLE",    1, SIM,  gps_disable[0], 0),
    AP_GROUPINFO("GPS_LAG_MS",     2, SIM,  gps_delay_ms[0], 100),
    AP_GROUPINFO("GPS_TYPE",       3, SIM,  gps_type[0],  GPS::Type::UBLOX),
    AP_GROUPINFO("GPS_BYTELOSS",   4, SIM,  gps_byteloss[0],  0),
    AP_GROUPINFO("GPS_NUMSATS",    5, SIM,  gps_numsats[0],   10),
    AP_GROUPINFO("GPS_GLITCH",     6, SIM,  gps_glitch[0],  0),
    AP_GROUPINFO("GPS_HZ",         7, SIM,  gps_hertz[0],  5),
    AP_GROUPINFO("GPS_DRIFTALT",   8, SIM,  gps_drift_alt[0], 0),
    AP_GROUPINFO("GPS_POS",        9, SIM,  gps_pos_offset[0], 0),
    AP_GROUPINFO("GPS_NOISE",     10, SIM,  gps_noise[0], 0),
    AP_GROUPINFO("GPS_LOCKTIME",  11, SIM,  gps_lock_time[0], 0),
    AP_GROUPINFO("GPS_ALT_OFS",   12, SIM,  gps_alt_offset[0], 0),
    AP_GROUPINFO("GPS_HDG",       13, SIM,  gps_hdg_enabled[0], SIM::GPS_HEADING_NONE),
    AP_GROUPINFO("GPS_ACC",       14, SIM,  gps_accuracy[0], 0.3),
    AP_GROUPINFO("GPS_VERR",      15, SIM,  gps_vel_err[0], 0),

    AP_GROUPINFO("GPS2_DISABLE",  30, SIM,  gps_disable[1], 1),
    AP_GROUPINFO("GPS2_LAG_MS",   31, SIM,  gps_delay_ms[1], 100),
    AP_GROUPINFO("GPS2_TYPE",     32, SIM,  gps_type[1],  GPS::Type::UBLOX),
    AP_GROUPINFO("GPS2_BYTELOS",  33, SIM,  gps_byteloss[1],  0),
    AP_GROUPINFO("GPS2_NUMSATS",  34, SIM,  gps_numsats[1],   10),
    AP_GROUPINFO("GPS2_GLTCH",    35, SIM,  gps_glitch[1],  0),
    AP_GROUPINFO("GPS2_HZ",       36, SIM,  gps_hertz[1],  5),
    AP_GROUPINFO("GPS2_DRFTALT",  37, SIM,  gps_drift_alt[1], 0),
    AP_GROUPINFO("GPS2_POS",      38, SIM,  gps_pos_offset[1], 0),
    AP_GROUPINFO("GPS2_NOISE",    39, SIM,  gps_noise[1], 0),
    AP_GROUPINFO("GPS2_LCKTIME",  40, SIM,  gps_lock_time[1], 0),
    AP_GROUPINFO("GPS2_ALT_OFS",  41, SIM,  gps_alt_offset[1], 0),
    AP_GROUPINFO("GPS2_HDG",      42, SIM,  gps_hdg_enabled[1], SIM::GPS_HEADING_NONE),
    AP_GROUPINFO("GPS2_ACC",      43, SIM,  gps_accuracy[1], 0.3),
    AP_GROUPINFO("GPS2_VERR",     44, SIM,  gps_vel_err[1], 0),

    AP_GROUPINFO("INIT_LAT_OFS",  45, SIM,  gps_init_lat_ofs, 0),
    AP_GROUPINFO("INIT_LON_OFS",  46, SIM,  gps_init_lon_ofs, 0),
    AP_GROUPINFO("INIT_ALT_OFS",  47, SIM,  gps_init_alt_ofs, 0),

    AP_GROUPEND
};
#endif  // HAL_SIM_GPS_ENABLED

// Mag SITL parameters
const AP_Param::GroupInfo SIM::var_mag[] = {
    AP_GROUPINFO("MAG_RND",        1, SIM,  mag_noise,   0),
    AP_GROUPINFO("MAG_MOT",        2, SIM,  mag_mot, 0),
    AP_GROUPINFO("MAG_DELAY",      3, SIM,  mag_delay, 0),
    AP_GROUPINFO("MAG_OFS",        4, SIM,  mag_ofs[0], 0),
    AP_GROUPINFO("MAG_ALY",        5, SIM,  mag_anomaly_ned, 0),
    AP_GROUPINFO("MAG_ALY_HGT",    6, SIM,  mag_anomaly_hgt, 1.0f),
    AP_GROUPINFO("MAG_DIA",        7, SIM,  mag_diag[0], 0),
    AP_GROUPINFO("MAG_ODI",        8, SIM,  mag_offdiag[0], 0),
    AP_GROUPINFO("MAG_ORIENT",     9, SIM,  mag_orient[0], 0),
    AP_GROUPINFO("MAG1_SCALING",  10, SIM,  mag_scaling[0], 1),
    AP_GROUPINFO("MAG1_DEVID",    11, SIM,  mag_devid[0], 97539),
    AP_GROUPINFO("MAG2_DEVID",    12, SIM,  mag_devid[1], 131874),
#if MAX_CONNECTED_MAGS > 2
    AP_GROUPINFO("MAG3_DEVID",    13, SIM,  mag_devid[2], 263178),
#endif
#if MAX_CONNECTED_MAGS > 3
    AP_GROUPINFO("MAG4_DEVID",    14, SIM,  mag_devid[3], 97283),
#endif
#if MAX_CONNECTED_MAGS > 4
    AP_GROUPINFO("MAG5_DEVID",    15, SIM,  mag_devid[4], 97795),
#endif
#if MAX_CONNECTED_MAGS > 5
    AP_GROUPINFO("MAG6_DEVID",    16, SIM,  mag_devid[5], 98051),
#endif
#if MAX_CONNECTED_MAGS > 6
    AP_GROUPINFO("MAG7_DEVID",    17, SIM,  mag_devid[6], 0),
#endif
#if MAX_CONNECTED_MAGS > 7
    AP_GROUPINFO("MAG8_DEVID",    18, SIM,  mag_devid[7], 0),
#endif
    AP_GROUPINFO("MAG1_FAIL",     26, SIM,  mag_fail[0], 0),
#if HAL_COMPASS_MAX_SENSORS > 1
    AP_GROUPINFO("MAG2_OFS",      19, SIM,  mag_ofs[1], 0),
    AP_GROUPINFO("MAG2_DIA",      20, SIM,  mag_diag[1], 0),
    AP_GROUPINFO("MAG2_ODI",      21, SIM,  mag_offdiag[1], 0),
    AP_GROUPINFO("MAG2_ORIENT",   22, SIM,  mag_orient[1], 0),
    AP_GROUPINFO("MAG2_FAIL",     27, SIM,  mag_fail[1], 0),
    AP_GROUPINFO("MAG2_SCALING",  28, SIM,  mag_scaling[1], 1),
#endif
#if HAL_COMPASS_MAX_SENSORS > 2
    AP_GROUPINFO("MAG3_OFS",      23, SIM,  mag_ofs[2], 0),
    AP_GROUPINFO("MAG3_DIA",      24, SIM,  mag_diag[2], 0),
    AP_GROUPINFO("MAG3_ODI",      25, SIM,  mag_offdiag[2], 0),
    AP_GROUPINFO("MAG3_FAIL",     29, SIM,  mag_fail[2], 0),
    AP_GROUPINFO("MAG3_SCALING",  30, SIM,  mag_scaling[2], 1),
    AP_GROUPINFO("MAG3_ORIENT",   36, SIM,  mag_orient[2], 0),
#endif
    AP_GROUPEND
};

#ifdef SFML_JOYSTICK
const AP_Param::GroupInfo SIM::var_sfml_joystick[] = {
    AP_GROUPINFO("SF_JS_STICK",    1, SIM,  sfml_joystick_id,   0),
    AP_GROUPINFO("SF_JS_AXIS1",    2, SIM,  sfml_joystick_axis[0], sf::Joystick::Axis::X),
    AP_GROUPINFO("SF_JS_AXIS2",    3, SIM,  sfml_joystick_axis[1], sf::Joystick::Axis::Y),
    AP_GROUPINFO("SF_JS_AXIS3",    4, SIM,  sfml_joystick_axis[2], sf::Joystick::Axis::Z),
    AP_GROUPINFO("SF_JS_AXIS4",    5, SIM,  sfml_joystick_axis[3], sf::Joystick::Axis::U),
    AP_GROUPINFO("SF_JS_AXIS5",    6, SIM,  sfml_joystick_axis[4], sf::Joystick::Axis::V),
    AP_GROUPINFO("SF_JS_AXIS6",    7, SIM,  sfml_joystick_axis[5], sf::Joystick::Axis::R),
    AP_GROUPINFO("SF_JS_AXIS7",    8, SIM,  sfml_joystick_axis[6], sf::Joystick::Axis::PovX),
    AP_GROUPINFO("SF_JS_AXIS8",    9, SIM,  sfml_joystick_axis[7], sf::Joystick::Axis::PovY),
    AP_GROUPEND
};
#endif //SFML_JOYSTICK

// INS SITL parameters
const AP_Param::GroupInfo SIM::var_ins[] = {
#if HAL_INS_TEMPERATURE_CAL_ENABLE
    AP_GROUPINFO("IMUT_START",    1, SIM, imu_temp_start,  25),
    AP_GROUPINFO("IMUT_END",      2, SIM, imu_temp_end, 45),
    AP_GROUPINFO("IMUT_TCONST",   3, SIM, imu_temp_tconst, 300),
    AP_GROUPINFO("IMUT_FIXED",    4, SIM, imu_temp_fixed, 0),
#endif
    AP_GROUPINFO("ACC1_BIAS",     5, SIM, accel_bias[0], 0),
#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("ACC2_BIAS",     6, SIM, accel_bias[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("ACC3_BIAS",     7, SIM, accel_bias[2], 0),
#endif
    AP_GROUPINFO("GYR1_RND",      8, SIM, gyro_noise[0],  0),
#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("GYR2_RND",      9, SIM, gyro_noise[1],  0),
#endif
#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("GYR3_RND",     10, SIM, gyro_noise[2],  0),
#endif
    AP_GROUPINFO("ACC1_RND",     11, SIM, accel_noise[0], 0),
#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("ACC2_RND",     12, SIM, accel_noise[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("ACC3_RND",     13, SIM, accel_noise[2], 0),
#endif
    AP_GROUPINFO("GYR1_SCALE",   14, SIM, gyro_scale[0], 0),
#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("GYR2_SCALE",   15, SIM, gyro_scale[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("GYR3_SCALE",   16, SIM, gyro_scale[2], 0),
#endif
    AP_GROUPINFO("ACCEL1_FAIL",  17, SIM, accel_fail[0],  0),
#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("ACCEL2_FAIL",  18, SIM, accel_fail[1],  0),
#endif
#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("ACCEL3_FAIL",  19, SIM, accel_fail[2],  0),
#endif
    AP_GROUPINFO("GYR_FAIL_MSK", 20, SIM, gyro_fail_mask,  0),
    AP_GROUPINFO("ACC_FAIL_MSK", 21, SIM, accel_fail_mask,  0),
    AP_GROUPINFO("ACC1_SCAL",    22, SIM, accel_scale[0], 0),
#if INS_MAX_INSTANCES > 1
    AP_GROUPINFO("ACC2_SCAL",    23, SIM, accel_scale[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    AP_GROUPINFO("ACC3_SCAL",    24, SIM, accel_scale[2], 0),
#endif
    AP_GROUPINFO("ACC_TRIM",     25, SIM, accel_trim, 0),

    // @Param: SAIL_TYPE
    // @DisplayName: Sailboat simulation sail type
    // @Description: 0: mainsail with sheet, 1: directly actuated wing
    AP_GROUPINFO("SAIL_TYPE",     26, SIM, sail_type, 0),

    // @Param: JSON_MASTER
    // @DisplayName: JSON master instance
    // @Description: the instance number to  take servos from
    AP_GROUPINFO("JSON_MASTER",     27, SIM, ride_along_master, 0),

    // @Param: OH_MASK
    // @DisplayName: SIM-on_hardware Output Enable Mask
    // @Description: channels which are passed through to actual hardware when running on actual hardware
    AP_GROUPINFO("OH_MASK",     28, SIM, on_hardware_output_enable_mask, 0),

    // the IMUT parameters must be last due to the enable parameters
#if HAL_INS_TEMPERATURE_CAL_ENABLE
    AP_SUBGROUPINFO(imu_tcal[0], "IMUT1_", 61, SIM, AP_InertialSensor::TCal),
#if INS_MAX_INSTANCES > 1
    AP_SUBGROUPINFO(imu_tcal[1], "IMUT2_", 62, SIM, AP_InertialSensor::TCal),
#endif
#if INS_MAX_INSTANCES > 2
    AP_SUBGROUPINFO(imu_tcal[2], "IMUT3_", 63, SIM, AP_InertialSensor::TCal),
#endif
#endif  // HAL_INS_TEMPERATURE_CAL_ENABLE
    AP_GROUPEND
};

const Location post_origin {
    518752066,
    146487830,
    0,
    Location::AltFrame::ABSOLUTE
};

/* report SITL state via MAVLink SIMSTATE*/
void SIM::simstate_send(mavlink_channel_t chan) const
{
    float yaw;

    // convert to same conventions as DCM
    yaw = state.yawDeg;
    if (yaw > 180) {
        yaw -= 360;
    }

    mavlink_msg_simstate_send(chan,
                              ToRad(state.rollDeg),
                              ToRad(state.pitchDeg),
                              ToRad(yaw),
                              state.xAccel,
                              state.yAccel,
                              state.zAccel,
                              radians(state.rollRate),
                              radians(state.pitchRate),
                              radians(state.yawRate),
                              state.latitude*1.0e7,
                              state.longitude*1.0e7);
}

/* report SITL state via MAVLink SIM_STATE */
void SIM::sim_state_send(mavlink_channel_t chan) const
{
    // convert to same conventions as DCM
    float yaw = state.yawDeg;
    if (yaw > 180) {
        yaw -= 360;
    }

    mavlink_msg_sim_state_send(chan,
            state.quaternion.q1,
            state.quaternion.q2,
            state.quaternion.q3,
            state.quaternion.q4,
            ToRad(state.rollDeg),
            ToRad(state.pitchDeg),
            ToRad(yaw),
            state.xAccel,
            state.yAccel,
            state.zAccel,
            radians(state.rollRate),
            radians(state.pitchRate),
            radians(state.yawRate),
            state.latitude*1.0e7,
            state.longitude*1.0e7,
            (float)state.altitude,
            0.0,
            0.0,
            state.speedN,
            state.speedE,
            state.speedD);
}

/* report SITL state to AP_Logger */
void SIM::Log_Write_SIMSTATE()
{
    float yaw;

    // convert to same conventions as DCM
    yaw = state.yawDeg;
    if (yaw > 180) {
        yaw -= 360;
    }

    struct log_AHRS pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SIMSTATE_MSG),
        time_us : AP_HAL::micros64(),
        roll    : (int16_t)(state.rollDeg*100),
        pitch   : (int16_t)(state.pitchDeg*100),
        yaw     : (uint16_t)(wrap_360_cd(yaw*100)),
        alt     : (float)state.altitude,
        lat     : (int32_t)(state.latitude*1.0e7),
        lng     : (int32_t)(state.longitude*1.0e7),
        q1      : state.quaternion.q1,
        q2      : state.quaternion.q2,
        q3      : state.quaternion.q3,
        q4      : state.quaternion.q4,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

/*
 convert a set of roll rates from earth frame to body frame
 output values are in radians/second
*/
void SIM::convert_body_frame(double rollDeg, double pitchDeg,
                              double rollRate, double pitchRate, double yawRate,
                              double *p, double *q, double *r)
{
    double phi, theta, phiDot, thetaDot, psiDot;

    phi = ToRad(rollDeg);
    theta = ToRad(pitchDeg);
    phiDot = ToRad(rollRate);
    thetaDot = ToRad(pitchRate);
    psiDot = ToRad(yawRate);

    *p = phiDot - psiDot*sin(theta);
    *q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta);
    *r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot;
}


/*
  convert angular velocities from body frame to
  earth frame.

  all inputs and outputs are in radians/s
*/
Vector3f SIM::convert_earth_frame(const Matrix3f &dcm, const Vector3f &gyro)
{
    float p = gyro.x;
    float q = gyro.y;
    float r = gyro.z;

    float phi, theta, psi;
    dcm.to_euler(&phi, &theta, &psi);

    float phiDot = p + tanf(theta)*(q*sinf(phi) + r*cosf(phi));
    float thetaDot = q*cosf(phi) - r*sinf(phi);
    if (fabsf(cosf(theta)) < 1.0e-20f) {
        theta += 1.0e-10f;
    }
    float psiDot = (q*sinf(phi) + r*cosf(phi))/cosf(theta);
    return Vector3f(phiDot, thetaDot, psiDot);
}

// get the rangefinder reading for the desired rotation, returns -1 for no data
float SIM::get_rangefinder(uint8_t instance) {
    if (instance < ARRAY_SIZE(state.rangefinder_m)) {
        return state.rangefinder_m[instance];
    }
    return -1;
};

float SIM::measure_distance_at_angle_bf(const Location &location, float angle) const
{
    // should we populate state.rangefinder_m[...] from this?
    Vector2f vehicle_pos_cm;
    if (!location.get_vector_xy_from_origin_NE(vehicle_pos_cm)) {
        // should probably use SITL variables...
        return 0.0f;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static uint64_t count = 0;

    if (count == 0) {
        unlink("/tmp/rayfile.scr");
        unlink("/tmp/intersectionsfile.scr");
    }

    count++;

    // the 1000 here is so the files don't grow unbounded
    const bool write_debug_files = count < 1000;

    FILE *rayfile = nullptr;
    if (write_debug_files) {
        rayfile = fopen("/tmp/rayfile.scr", "a");
    }
#endif

    // cast a ray from location out 200m...
    Location location2 = location;
    location2.offset_bearing(wrap_180(angle + state.yawDeg), 200);
    Vector2f ray_endpos_cm;
    if (!location2.get_vector_xy_from_origin_NE(ray_endpos_cm)) {
        // should probably use SITL variables...
        return 0.0f;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (rayfile != nullptr) {
        ::fprintf(rayfile, "map icon %f %f barrell\n", location2.lat*1e-7, location2.lng*1e-7);
        fclose(rayfile);
    }

    // setup a grid of posts
    FILE *postfile = nullptr;
    FILE *intersectionsfile = nullptr;
    if (write_debug_files) {
        static bool postfile_written;
        if (!postfile_written) {
            ::fprintf(stderr, "Writing /tmp/post-locations.scr\n");
            postfile_written = true;
            postfile = fopen("/tmp/post-locations.scr", "w");
        }
        intersectionsfile = fopen("/tmp/intersections.scr", "a");
    }
#endif

    const float radius_cm = 100.0f;
    float min_dist_cm = 1000000.0;
    const uint8_t num_post_offset = 10;
    for (int8_t x=-num_post_offset; x<num_post_offset; x++) {
        for (int8_t y=-num_post_offset; y<num_post_offset; y++) {
            Location post_location = post_origin;
            post_location.offset(x*10+3, y*10+2);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            if (postfile != nullptr) {
                ::fprintf(postfile, "map circle %f %f %f blue\n", post_location.lat*1e-7, post_location.lng*1e-7, radius_cm/100.0);
            }
#endif
            Vector2f post_position_cm;
            if (!post_location.get_vector_xy_from_origin_NE(post_position_cm)) {
                // should probably use SITL variables...
                return 0.0f;
            }
            Vector2f intersection_point_cm;
            if (Vector2f::circle_segment_intersection(ray_endpos_cm, vehicle_pos_cm, post_position_cm, radius_cm, intersection_point_cm)) {
                float dist_cm = (intersection_point_cm-vehicle_pos_cm).length();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                if (intersectionsfile != nullptr) {
                    Location intersection_point = location;
                    intersection_point.offset(intersection_point_cm.x/100.0,
                                              intersection_point_cm.y/100.0);
                    ::fprintf(intersectionsfile,
                              "map icon %f %f barrell\n",
                              intersection_point.lat*1e-7,
                              intersection_point.lng*1e-7);
                }
#endif
                if (dist_cm < min_dist_cm) {
                    min_dist_cm = dist_cm;
                }
            }
        }
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (postfile != nullptr) {
        fclose(postfile);
    }
    if (intersectionsfile != nullptr) {
        fclose(intersectionsfile);
    }
#endif

    // ::fprintf(stderr, "Distance @%f = %fm\n", angle, min_dist_cm/100.0f);
    return min_dist_cm / 100.0f;
}

} // namespace SITL

namespace AP {

SITL::SIM *sitl()
{
    return SITL::SIM::get_singleton();
}

};

#endif // AP_SIM_ENABLED
