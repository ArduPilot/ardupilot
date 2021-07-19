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

#include "SITL.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Logger/AP_Logger.h>

#ifdef SFML_JOYSTICK
  #ifdef HAVE_SFML_GRAPHICS_HPP
    #include <SFML/Window/Joystick.hpp>
  #elif HAVE_SFML_GRAPHIC_H
    #include <SFML/Window/Joystick.h>
  #endif
#endif // SFML_JOYSTICK

extern const AP_HAL::HAL& hal;

namespace SITL {

SITL *SITL::_singleton = nullptr;

// table of user settable parameters
const AP_Param::GroupInfo SITL::var_info[] = {
    
    AP_GROUPINFO("DRIFT_SPEED",    5, SITL,  drift_speed, 0.05f),
    AP_GROUPINFO("DRIFT_TIME",     6, SITL,  drift_time,  5),
    AP_GROUPINFO("ENGINE_MUL",     8, SITL,  engine_mul,  1),
    AP_GROUPINFO("WIND_SPD",       9, SITL,  wind_speed,  0),
    AP_GROUPINFO("WIND_DIR",      10, SITL,  wind_direction,  180),
    AP_GROUPINFO("WIND_TURB",     11, SITL,  wind_turbulance,  0),
    AP_GROUPINFO("SERVO_SPEED",   16, SITL,  servo_speed,  0.14),
    AP_GROUPINFO("BATT_VOLTAGE",  19, SITL,  batt_voltage,  12.6f),
    AP_GROUPINFO("BATT_CAP_AH",   20, SITL,  batt_capacity_ah,  0),
    AP_GROUPINFO("SONAR_GLITCH",  23, SITL,  sonar_glitch, 0),
    AP_GROUPINFO("SONAR_RND",     24, SITL,  sonar_noise, 0),
    AP_GROUPINFO("RC_FAIL",       25, SITL,  rc_fail, 0),
    AP_GROUPINFO("FLOAT_EXCEPT",  28, SITL,  float_exception, 1),
    AP_GROUPINFO("SONAR_SCALE",   32, SITL,  sonar_scale, 12.1212f),
    AP_GROUPINFO("FLOW_ENABLE",   33, SITL,  flow_enable, 0),
    AP_GROUPINFO("TERRAIN",       34, SITL,  terrain_enable, 1),
    AP_GROUPINFO("FLOW_RATE",     35, SITL,  flow_rate, 10),
    AP_GROUPINFO("FLOW_DELAY",    36, SITL,  flow_delay, 0),
    AP_GROUPINFO("WIND_DELAY",    40, SITL,  wind_delay, 0),
    AP_GROUPINFO("ADSB_COUNT",    45, SITL,  adsb_plane_count, -1),
    AP_GROUPINFO("ADSB_RADIUS",   46, SITL,  adsb_radius_m, 10000),
    AP_GROUPINFO("ADSB_ALT",      47, SITL,  adsb_altitude_m, 1000),
    AP_GROUPINFO("PIN_MASK",      50, SITL,  pin_mask, 0),
    AP_GROUPINFO("ADSB_TX",       51, SITL,  adsb_tx, 0),
    AP_GROUPINFO("SPEEDUP",       52, SITL,  speedup, -1),
    AP_GROUPINFO("IMU_POS",       53, SITL,  imu_pos_offset, 0),
    AP_SUBGROUPEXTENSION("",      54, SITL,  var_ins),
    AP_GROUPINFO("SONAR_POS",     55, SITL,  rngfnd_pos_offset, 0),
    AP_GROUPINFO("FLOW_POS",      56, SITL,  optflow_pos_offset, 0),
    AP_GROUPINFO("ENGINE_FAIL",   58, SITL,  engine_fail,  0),
    AP_SUBGROUPINFO(shipsim, "SHIP_", 59, SITL, ShipSim),
    AP_SUBGROUPEXTENSION("",      60, SITL,  var_mag),
    AP_SUBGROUPEXTENSION("",      61, SITL,  var_gps),
    AP_SUBGROUPEXTENSION("",      62, SITL,  var_info3),
    AP_SUBGROUPEXTENSION("",      63, SITL,  var_info2),
    AP_GROUPEND
};

// second table of user settable parameters for SITL. 
const AP_Param::GroupInfo SITL::var_info2[] = {
    AP_GROUPINFO("TEMP_START",   1, SITL,  temp_start,  25),
    AP_GROUPINFO("TEMP_FLIGHT",  2, SITL,  temp_flight, 35),
    AP_GROUPINFO("TEMP_TCONST",  3, SITL,  temp_tconst, 30),
    AP_GROUPINFO("TEMP_BFACTOR", 4, SITL,  temp_baro_factor, 0),

    AP_GROUPINFO("WIND_DIR_Z",  10, SITL,  wind_dir_z,     0),
    AP_GROUPINFO("WIND_T"      ,15, SITL,  wind_type, SITL::WIND_TYPE_SQRT),
    AP_GROUPINFO("WIND_T_ALT"  ,16, SITL,  wind_type_alt, 60),
    AP_GROUPINFO("WIND_T_COEF", 17, SITL,  wind_type_coef, 0.01f),
    AP_GROUPINFO("RC_CHANCOUNT",21, SITL,  rc_chancount, 16),
    // @Group: SPR_
    // @Path: ./SIM_Sprayer.cpp
    AP_SUBGROUPINFO(sprayer_sim, "SPR_", 22, SITL, Sprayer),
    // @Group: GRPS_
    // @Path: ./SIM_Gripper_Servo.cpp
    AP_SUBGROUPINFO(gripper_sim, "GRPS_", 23, SITL, Gripper_Servo),
    // @Group: GRPE_
    // @Path: ./SIM_Gripper_EPM.cpp
    AP_SUBGROUPINFO(gripper_epm_sim, "GRPE_", 24, SITL, Gripper_EPM),

    // weight on wheels pin
    AP_GROUPINFO("WOW_PIN",     25, SITL,  wow_pin, -1),

    // vibration frequencies on each axis
    AP_GROUPINFO("VIB_FREQ",   26, SITL,  vibe_freq, 0),

    // @Path: ./SIM_Parachute.cpp
    AP_SUBGROUPINFO(parachute_sim, "PARA_", 27, SITL, Parachute),

    // enable bandwidth limitting on telemetry ports:
    AP_GROUPINFO("BAUDLIMIT_EN",   28, SITL,  telem_baudlimit_enable, 0),

    // @Group: PLD_
    // @Path: ./SIM_Precland.cpp
    AP_SUBGROUPINFO(precland_sim, "PLD_", 29, SITL, SIM_Precland),

    // apply a force to the vehicle over a period of time:
    AP_GROUPINFO("SHOVE_X",     30, SITL,  shove.x, 0),
    AP_GROUPINFO("SHOVE_Y",     31, SITL,  shove.y, 0),
    AP_GROUPINFO("SHOVE_Z",     32, SITL,  shove.z, 0),
    AP_GROUPINFO("SHOVE_TIME",  33, SITL,  shove.t, 0),
    
    // optical flow sensor measurement noise in rad/sec
    AP_GROUPINFO("FLOW_RND",   34, SITL,  flow_noise,  0.05f),

    AP_GROUPINFO("TWIST_X",     37, SITL,  twist.x, 0),
    AP_GROUPINFO("TWIST_Y",     38, SITL,  twist.y, 0),
    AP_GROUPINFO("TWIST_Z",     39, SITL,  twist.z, 0),
    AP_GROUPINFO("TWIST_TIME",  40, SITL,  twist.t, 0),

    AP_GROUPINFO("GND_BEHAV",   41, SITL,  gnd_behav, -1),

    // sailboat wave and tide simulation parameters
    AP_GROUPINFO("WAVE_ENABLE", 44, SITL,  wave.enable, 0.0f),
    AP_GROUPINFO("WAVE_LENGTH", 45, SITL,  wave.length, 10.0f),
    AP_GROUPINFO("WAVE_AMP",    46, SITL,  wave.amp, 0.5f),
    AP_GROUPINFO("WAVE_DIR",    47, SITL,  wave.direction, 0.0f),
    AP_GROUPINFO("WAVE_SPEED",  48, SITL,  wave.speed, 0.5f),
    AP_GROUPINFO("TIDE_DIR",    49, SITL,  tide.direction, 0.0f),
    AP_GROUPINFO("TIDE_SPEED",  50, SITL,  tide.speed, 0.0f),

    // the following coordinates are for CMAC, in Canberra
    AP_GROUPINFO("OPOS_LAT",    51, SITL,  opos.lat, -35.363261f),
    AP_GROUPINFO("OPOS_LNG",    52, SITL,  opos.lng, 149.165230f),
    AP_GROUPINFO("OPOS_ALT",    53, SITL,  opos.alt, 584.0f),
    AP_GROUPINFO("OPOS_HDG",    54, SITL,  opos.hdg, 353.0f),

    // extra delay per main loop
    AP_GROUPINFO("LOOP_DELAY",  55, SITL,  loop_delay, 0),

    // @Path: ./SIM_Buzzer.cpp
    AP_SUBGROUPINFO(buzzer_sim, "BZ_", 56, SITL, Buzzer),

    // @Path: ./SIM_ToneAlarm.cpp
    AP_SUBGROUPINFO(tonealarm_sim, "TA_", 57, SITL, ToneAlarm),

    AP_GROUPINFO("EFI_TYPE",    58, SITL,  efi_type,  SITL::EFI_TYPE_NONE),

    AP_GROUPINFO("SAFETY_STATE",    59, SITL,  _safety_switch_state, 0),

    // max motor vibration frequency
    AP_GROUPINFO("VIB_MOT_MAX", 61, SITL,  vibe_motor, 0.0f),
    // minimum throttle for simulated ins noise
    AP_GROUPINFO("INS_THR_MIN", 62, SITL,  ins_noise_throttle_min, 0.1f),
    // amplitude scaling of motor noise relative to gyro/accel noise
    AP_GROUPINFO("VIB_MOT_MULT", 63, SITL,  vibe_motor_scale, 1.0f),

    AP_GROUPEND

};

// third table of user settable parameters for SITL. 
const AP_Param::GroupInfo SITL::var_info3[] = {
    AP_GROUPINFO("ODOM_ENABLE",   1, SITL,  odom_enable, 0),

    AP_GROUPINFO("LED_LAYOUT",    11, SITL, led_layout, 0),

    // Scenario for thermalling simulation, for soaring
    AP_GROUPINFO("THML_SCENARI",  12, SITL,  thermal_scenario, 0),

    // vicon sensor position (position offsets in body frame)
    AP_GROUPINFO("VICON_POS",     14, SITL,  vicon_pos_offset, 0),

    // Buyoancy for submarines
    AP_GROUPINFO_FRAME("BUOYANCY", 15, SITL, buoyancy, 1, AP_PARAM_FRAME_SUB),

    // vicon glitch in NED frame
    AP_GROUPINFO("VICON_GLIT",    16, SITL,  vicon_glitch, 0),

    // vicon failure
    AP_GROUPINFO("VICON_FAIL",    17, SITL,  vicon_fail, 0),

    // vicon yaw (in earth frame)
    AP_GROUPINFO("VICON_YAW",     18, SITL,  vicon_yaw, 0),

    // vicon yaw error in degrees (added to reported yaw sent to vehicle)
    AP_GROUPINFO("VICON_YAWERR",  19, SITL,  vicon_yaw_error, 0),

    // vicon message type mask
    AP_GROUPINFO("VICON_TMASK",   20, SITL,  vicon_type_mask, 3),

    // vicon velocity glitch in NED frame
    AP_GROUPINFO("VICON_VGLI",    21, SITL,  vicon_vel_glitch, 0),

    AP_GROUPINFO("RATE_HZ",  22, SITL,  loop_rate_hz, 1200),

    // count of simulated IMUs
    AP_GROUPINFO("IMU_COUNT",    23, SITL,  imu_count,  2),

    // @Path: ./SIM_RichenPower.cpp
    AP_SUBGROUPINFO(richenpower_sim, "RICH_", 31, SITL, RichenPower),

    // @Path: ./SIM_IntelligentEnergy24.cpp
    AP_SUBGROUPINFO(ie24_sim, "IE24_", 32, SITL, IntelligentEnergy24),

    // user settable barometer parameters
    AP_GROUPINFO("BARO_COUNT",    33, SITL,  baro_count, 2),

    AP_SUBGROUPINFO(baro[0], "BARO_", 34, SITL, SITL::BaroParm),
    AP_SUBGROUPINFO(baro[1], "BAR2_", 35, SITL, SITL::BaroParm),
    AP_SUBGROUPINFO(baro[2], "BAR3_", 36, SITL, SITL::BaroParm),

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

    AP_GROUPINFO("ESC_TELEM", 40, SITL, esc_telem, 1),

    // user settable parameters for the 1st airspeed sensor
    AP_GROUPINFO("ARSPD_RND",     50, SITL,  arspd_noise[0], 2.0),
    AP_GROUPINFO("ARSPD_OFS",     51, SITL,  arspd_offset[0], 2013),
    AP_GROUPINFO("ARSPD_FAIL",    52, SITL,  arspd_fail[0], 0),
    AP_GROUPINFO("ARSPD_FAILP",   53, SITL,  arspd_fail_pressure[0], 0),
    AP_GROUPINFO("ARSPD_PITOT",   54, SITL,  arspd_fail_pitot_pressure[0], 0),

    // user settable parameters for the 2nd airspeed sensor
    AP_GROUPINFO("ARSPD2_RND",    56, SITL,  arspd_noise[1], 2.0),
    AP_GROUPINFO("ARSPD2_OFS",    57, SITL,  arspd_offset[1], 2013),
    AP_GROUPINFO("ARSPD2_FAIL",   58, SITL,  arspd_fail[1], 0),
    AP_GROUPINFO("ARSPD2_FAILP",  59, SITL,  arspd_fail_pressure[1], 0),
    AP_GROUPINFO("ARSPD2_PITOT",  60, SITL,  arspd_fail_pitot_pressure[1], 0),

    // user settable common airspeed parameters
    AP_GROUPINFO("ARSPD_SIGN",    62, SITL,  arspd_signflip, 0),

#ifdef SFML_JOYSTICK
    AP_SUBGROUPEXTENSION("",      63, SITL,  var_sfml_joystick),
#endif // SFML_JOYSTICK

    AP_GROUPEND
};

// user settable parameters for the barometers
const AP_Param::GroupInfo SITL::BaroParm::var_info[] = {
    AP_GROUPINFO("RND",      1, SITL::BaroParm,  noise, 0.2f),
    AP_GROUPINFO("DRIFT",    2, SITL::BaroParm,  drift, 0),
    AP_GROUPINFO("DISABLE",  3, SITL::BaroParm,  disable, 0),
    AP_GROUPINFO("GLITCH",   4, SITL::BaroParm,  glitch, 0),
    AP_GROUPINFO("FREEZE",   5, SITL::BaroParm,  freeze, 0),
    AP_GROUPINFO("DELAY",    6, SITL::BaroParm,  delay, 0),

    // wind coeffients
    AP_GROUPINFO("WCF_FWD", 7,  SITL::BaroParm, wcof_xp, 0.0),
    AP_GROUPINFO("WCF_BAK", 8,  SITL::BaroParm, wcof_xn, 0.0),
    AP_GROUPINFO("WCF_RGT", 9,  SITL::BaroParm, wcof_yp, 0.0),
    AP_GROUPINFO("WCF_LFT", 10, SITL::BaroParm, wcof_yn, 0.0),
    AP_GROUPEND
};
    
// GPS SITL parameters
const AP_Param::GroupInfo SITL::var_gps[] = {
    AP_GROUPINFO("GPS_DISABLE",    1, SITL,  gps_disable[0], 0),
    AP_GROUPINFO("GPS_DELAY",      2, SITL,  gps_delay[0],   1),
    AP_GROUPINFO("GPS_TYPE",       3, SITL,  gps_type[0],  SITL::GPS_TYPE_UBLOX),
    AP_GROUPINFO("GPS_BYTELOSS",   4, SITL,  gps_byteloss[0],  0),
    AP_GROUPINFO("GPS_NUMSATS",    5, SITL,  gps_numsats[0],   10),
    AP_GROUPINFO("GPS_GLITCH",     6, SITL,  gps_glitch[0],  0),
    AP_GROUPINFO("GPS_HZ",         7, SITL,  gps_hertz[0],  5),
    AP_GROUPINFO("GPS_DRIFTALT",   8, SITL,  gps_drift_alt[0], 0),
    AP_GROUPINFO("GPS_POS",        9, SITL,  gps_pos_offset[0], 0),
    AP_GROUPINFO("GPS_NOISE",     10, SITL,  gps_noise[0], 0),
    AP_GROUPINFO("GPS_LOCKTIME",  11, SITL,  gps_lock_time[0], 0),
    AP_GROUPINFO("GPS_ALT_OFS",   12, SITL,  gps_alt_offset[0], 0),
    AP_GROUPINFO("GPS_HDG",       13, SITL,  gps_hdg_enabled[0], SITL::GPS_HEADING_NONE),
    AP_GROUPINFO("GPS_ACC",       14, SITL,  gps_accuracy[0], 0.3),
    AP_GROUPINFO("GPS_VERR",      15, SITL,  gps_vel_err[0], 0),

    AP_GROUPINFO("GPS2_DISABLE",  30, SITL,  gps_disable[1], 1),
    AP_GROUPINFO("GPS2_DELAY",    31, SITL,  gps_delay[1],   1),
    AP_GROUPINFO("GPS2_TYPE",     32, SITL,  gps_type[1],  SITL::GPS_TYPE_UBLOX),
    AP_GROUPINFO("GPS2_BYTELOS",  33, SITL,  gps_byteloss[1],  0),
    AP_GROUPINFO("GPS2_NUMSATS",  34, SITL,  gps_numsats[1],   10),
    AP_GROUPINFO("GPS2_GLTCH",    35, SITL,  gps_glitch[1],  0),
    AP_GROUPINFO("GPS2_HZ",       36, SITL,  gps_hertz[1],  5),
    AP_GROUPINFO("GPS2_DRFTALT",  37, SITL,  gps_drift_alt[1], 0),
    AP_GROUPINFO("GPS2_POS",      38, SITL,  gps_pos_offset[1], 0),
    AP_GROUPINFO("GPS2_NOISE",    39, SITL,  gps_noise[1], 0),
    AP_GROUPINFO("GPS2_LCKTIME",  40, SITL,  gps_lock_time[1], 0),
    AP_GROUPINFO("GPS2_ALT_OFS",  41, SITL,  gps_alt_offset[1], 0),
    AP_GROUPINFO("GPS2_HDG",      42, SITL,  gps_hdg_enabled[1], SITL::GPS_HEADING_NONE),
    AP_GROUPINFO("GPS2_ACC",      43, SITL,  gps_accuracy[1], 0.3),
    AP_GROUPINFO("GPS2_VERR",     44, SITL,  gps_vel_err[1], 0),

    AP_GROUPINFO("INIT_LAT_OFS",  45, SITL,  gps_init_lat_ofs, 0),
    AP_GROUPINFO("INIT_LON_OFS",  46, SITL,  gps_init_lon_ofs, 0),
    AP_GROUPINFO("INIT_ALT_OFS",  47, SITL,  gps_init_alt_ofs, 0),

    AP_GROUPEND
};

// Mag SITL parameters
const AP_Param::GroupInfo SITL::var_mag[] = {
    AP_GROUPINFO("MAG_RND",        1, SITL,  mag_noise,   0),
    AP_GROUPINFO("MAG_MOT",        2, SITL,  mag_mot, 0),
    AP_GROUPINFO("MAG_DELAY",      3, SITL,  mag_delay, 0),
    AP_GROUPINFO("MAG_OFS",        4, SITL,  mag_ofs[0], 0),
    AP_GROUPINFO("MAG_ALY",        5, SITL,  mag_anomaly_ned, 0),
    AP_GROUPINFO("MAG_ALY_HGT",    6, SITL,  mag_anomaly_hgt, 1.0f),
    AP_GROUPINFO("MAG_DIA",        7, SITL,  mag_diag[0], 0),
    AP_GROUPINFO("MAG_ODI",        8, SITL,  mag_offdiag[0], 0),
    AP_GROUPINFO("MAG_ORIENT",     9, SITL,  mag_orient[0], 0),
    AP_GROUPINFO("MAG1_SCALING",  10, SITL,  mag_scaling[0], 1),
    AP_GROUPINFO("MAG1_DEVID",    11, SITL,  mag_devid[0], 97539),
    AP_GROUPINFO("MAG2_DEVID",    12, SITL,  mag_devid[1], 131874),
    AP_GROUPINFO("MAG3_DEVID",    13, SITL,  mag_devid[2], 263178),
    AP_GROUPINFO("MAG4_DEVID",    14, SITL,  mag_devid[3], 97283),
    AP_GROUPINFO("MAG5_DEVID",    15, SITL,  mag_devid[4], 97795),
    AP_GROUPINFO("MAG6_DEVID",    16, SITL,  mag_devid[5], 98051),
    AP_GROUPINFO("MAG7_DEVID",    17, SITL,  mag_devid[6], 0),
    AP_GROUPINFO("MAG8_DEVID",    18, SITL,  mag_devid[7], 0),
    AP_GROUPINFO("MAG1_FAIL",     26, SITL,  mag_fail[0], 0),
#if HAL_COMPASS_MAX_SENSORS > 1
    AP_GROUPINFO("MAG2_OFS",      19, SITL,  mag_ofs[1], 0),
    AP_GROUPINFO("MAG2_DIA",      20, SITL,  mag_diag[1], 0),
    AP_GROUPINFO("MAG2_ODI",      21, SITL,  mag_offdiag[1], 0),
    AP_GROUPINFO("MAG2_ORIENT",   22, SITL,  mag_orient[1], 0),
    AP_GROUPINFO("MAG2_FAIL",     27, SITL,  mag_fail[1], 0),
    AP_GROUPINFO("MAG2_SCALING",  28, SITL,  mag_scaling[1], 1),
#endif
#if HAL_COMPASS_MAX_SENSORS > 2
    AP_GROUPINFO("MAG3_OFS",      23, SITL,  mag_ofs[2], 0),
    AP_GROUPINFO("MAG3_DIA",      24, SITL,  mag_diag[2], 0),
    AP_GROUPINFO("MAG3_ODI",      25, SITL,  mag_offdiag[2], 0),
    AP_GROUPINFO("MAG3_FAIL",     29, SITL,  mag_fail[2], 0),
    AP_GROUPINFO("MAG3_SCALING",  30, SITL,  mag_scaling[2], 1),
    AP_GROUPINFO("MAG3_ORIENT",   36, SITL,  mag_orient[2], 0),
#endif
    AP_GROUPEND
};

#ifdef SFML_JOYSTICK
const AP_Param::GroupInfo SITL::var_sfml_joystick[] = {
    AP_GROUPINFO("SF_JS_STICK",    1, SITL,  sfml_joystick_id,   0),
    AP_GROUPINFO("SF_JS_AXIS1",    2, SITL,  sfml_joystick_axis[0], sf::Joystick::Axis::X),
    AP_GROUPINFO("SF_JS_AXIS2",    3, SITL,  sfml_joystick_axis[1], sf::Joystick::Axis::Y),
    AP_GROUPINFO("SF_JS_AXIS3",    4, SITL,  sfml_joystick_axis[2], sf::Joystick::Axis::Z),
    AP_GROUPINFO("SF_JS_AXIS4",    5, SITL,  sfml_joystick_axis[3], sf::Joystick::Axis::U),
    AP_GROUPINFO("SF_JS_AXIS5",    6, SITL,  sfml_joystick_axis[4], sf::Joystick::Axis::V),
    AP_GROUPINFO("SF_JS_AXIS6",    7, SITL,  sfml_joystick_axis[5], sf::Joystick::Axis::R),
    AP_GROUPINFO("SF_JS_AXIS7",    8, SITL,  sfml_joystick_axis[6], sf::Joystick::Axis::PovX),
    AP_GROUPINFO("SF_JS_AXIS8",    9, SITL,  sfml_joystick_axis[7], sf::Joystick::Axis::PovY),
    AP_GROUPEND
};
#endif //SFML_JOYSTICK

// INS SITL parameters
const AP_Param::GroupInfo SITL::var_ins[] = {
    AP_GROUPINFO("IMUT_START",    1, SITL, imu_temp_start,  25),
    AP_GROUPINFO("IMUT_END",      2, SITL, imu_temp_end, 45),
    AP_GROUPINFO("IMUT_TCONST",   3, SITL, imu_temp_tconst, 300),
    AP_GROUPINFO("IMUT_FIXED",    4, SITL, imu_temp_fixed, 0),
    AP_GROUPINFO("ACC1_BIAS",     5, SITL, accel_bias[0], 0),
    AP_GROUPINFO("ACC2_BIAS",     6, SITL, accel_bias[1], 0),
    AP_GROUPINFO("ACC3_BIAS",     7, SITL, accel_bias[2], 0),
    AP_GROUPINFO("GYR1_RND",      8, SITL, gyro_noise[0],  0),
    AP_GROUPINFO("GYR2_RND",      9, SITL, gyro_noise[1],  0),
    AP_GROUPINFO("GYR3_RND",     10, SITL, gyro_noise[2],  0),
    AP_GROUPINFO("ACC1_RND",     11, SITL, accel_noise[0], 0),
    AP_GROUPINFO("ACC2_RND",     12, SITL, accel_noise[1], 0),
    AP_GROUPINFO("ACC3_RND",     13, SITL, accel_noise[2], 0),
    AP_GROUPINFO("GYR1_SCALE",   14, SITL, gyro_scale[0], 0),
    AP_GROUPINFO("GYR2_SCALE",   15, SITL, gyro_scale[1], 0),
    AP_GROUPINFO("GYR3_SCALE",   16, SITL, gyro_scale[2], 0),
    AP_GROUPINFO("ACCEL1_FAIL",  17, SITL, accel_fail[0],  0),
    AP_GROUPINFO("ACCEL2_FAIL",  18, SITL, accel_fail[1],  0),
    AP_GROUPINFO("ACCEL3_FAIL",  19, SITL, accel_fail[2],  0),
    AP_GROUPINFO("GYR_FAIL_MSK", 20, SITL, gyro_fail_mask,  0),
    AP_GROUPINFO("ACC_FAIL_MSK", 21, SITL, accel_fail_mask,  0),
    AP_GROUPINFO("ACC1_SCAL",    22, SITL, accel_scale[0], 0),
    AP_GROUPINFO("ACC2_SCAL",    23, SITL, accel_scale[1], 0),
    AP_GROUPINFO("ACC3_SCAL",    24, SITL, accel_scale[2], 0),
    AP_GROUPINFO("ACC_TRIM",     25, SITL, accel_trim, 0),

    // @Param: SAIL_TYPE
    // @DisplayName: Sailboat simulation sail type
    // @Description: 0: mainsail with sheet, 1: directly actuated wing
    AP_GROUPINFO("SAIL_TYPE",     26, SITL, sail_type, 0),


    // the IMUT parameters must be last due to the enable parameters
    AP_SUBGROUPINFO(imu_tcal[0], "IMUT1_", 61, SITL, AP_InertialSensor::TCal),
    AP_SUBGROUPINFO(imu_tcal[1], "IMUT2_", 62, SITL, AP_InertialSensor::TCal),
    AP_SUBGROUPINFO(imu_tcal[2], "IMUT3_", 63, SITL, AP_InertialSensor::TCal),
    AP_GROUPEND
};
    
/* report SITL state via MAVLink SIMSTATE*/
void SITL::simstate_send(mavlink_channel_t chan) const
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
void SITL::sim_state_send(mavlink_channel_t chan) const
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
void SITL::Log_Write_SIMSTATE()
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
void SITL::convert_body_frame(double rollDeg, double pitchDeg,
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
Vector3f SITL::convert_earth_frame(const Matrix3f &dcm, const Vector3f &gyro)
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
float SITL::get_rangefinder(uint8_t instance) {
    if (instance < RANGEFINDER_MAX_INSTANCES) {
        return state.rangefinder_m[instance];
    }
    return -1;
};

} // namespace SITL


namespace AP {

SITL::SITL *sitl()
{
    return SITL::SITL::get_singleton();
}

};

#endif // CONFIG_HAL_BOARD
