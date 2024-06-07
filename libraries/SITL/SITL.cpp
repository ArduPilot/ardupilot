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

#if AP_SIM_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

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

#include "SIM_StratoBlimp.h"
#include "SIM_Glider.h"

extern const AP_HAL::HAL& hal;

#ifndef SIM_RATE_HZ_DEFAULT
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SIM_RATE_HZ_DEFAULT 1200
#else
#define SIM_RATE_HZ_DEFAULT 400
#endif
#endif

#if (CONFIG_HAL_BOARD != HAL_BOARD_SITL)
// For on-hardware, set allowed relay channels to zero.
// Requires user to change the param to allow hadware access.
#define SIM_DEFAULT_ENABLED_RELAY_CHANNELS 0
#else
// For SITL, set allowed relay channels to the full mask.
#define SIM_DEFAULT_ENABLED_RELAY_CHANNELS UINT16_MAX
#endif

namespace SITL {

SIM *SIM::_singleton = nullptr;

// table of user settable parameters
const AP_Param::GroupInfo SIM::var_info[] = {
    
    AP_GROUPINFO("DRIFT_SPEED",    5, SIM,  drift_speed, 0.05f),
    AP_GROUPINFO("DRIFT_TIME",     6, SIM,  drift_time,  5),
    AP_GROUPINFO("ENGINE_MUL",     8, SIM,  engine_mul,  1),
    // @Param: WIND_SPD
    // @DisplayName: Simulated Wind speed
    // @Description: Allows you to emulate wind in sim
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("WIND_SPD",       9, SIM,  wind_speed,  0),
    // @Param: WIND_DIR
    // @DisplayName: Simulated Wind direction
    // @Description: Allows you to set wind direction (true deg) in sim
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("WIND_DIR",      10, SIM,  wind_direction,  180),

    // @Param: WIND_TURB
    // @DisplayName: Simulated Wind variation
    // @Description: Allows you to emulate random wind variations in sim
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("WIND_TURB",     11, SIM,  wind_turbulance,  0),

    // @Param: WIND_TC
    // @DisplayName: Wind variation time constant
    // @Description: this controls the time over which wind changes take effect
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("WIND_TC",       12, SIM,  wind_change_tc,  5),

    // @Group: SERVO_
    // @Path: ./ServoModel.cpp
    AP_SUBGROUPINFO(servo, "SERVO_", 16, SIM, ServoParams),

    AP_GROUPINFO("SONAR_ROT",     17, SIM,  sonar_rot, Rotation::ROTATION_PITCH_270),
    // @Param: BATT_VOLTAGE
    // @DisplayName: Simulated battery voltage
    // @Description: Simulated battery (constant) voltage
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("BATT_VOLTAGE",  19, SIM,  batt_voltage,  12.6f),
    AP_GROUPINFO("BATT_CAP_AH",   20, SIM,  batt_capacity_ah,  0),
    AP_GROUPINFO("SONAR_GLITCH",  23, SIM,  sonar_glitch, 0),
    AP_GROUPINFO("SONAR_RND",     24, SIM,  sonar_noise, 0),
    // @Param: RC_FAIL
    // @DisplayName: Simulated RC signal failure
    // @Description: Allows you to emulate rc failures in sim
    // @Values: 0:Disabled,1:No RC pusles,2:All Channels neutral except Throttle is 950us
    // @User: Advanced
    AP_GROUPINFO("RC_FAIL",       25, SIM,  rc_fail, 0),
    // @Param: FLOAT_EXCEPT
    // @DisplayName: Generate floating point exceptions
    // @Description: If set, if a numerical error occurs SITL will die with a floating point exception.
    // @User: Advanced
    AP_GROUPINFO("FLOAT_EXCEPT",  28, SIM,  float_exception, 1),

    // @Param: CAN_SRV_MSK
    // @DisplayName: Mask of CAN servos/ESCs
    // @Description: The set of actuators controlled externally by CAN SITL AP_Periph
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32
    // @User: Advanced
    AP_GROUPINFO("CAN_SRV_MSK",   29, SIM,  can_servo_mask, 0),

#if HAL_NUM_CAN_IFACES > 0
    // @Param: CAN_TYPE1
    // @DisplayName: transport type for first CAN interface
    // @Description: transport type for first CAN interface
    // @Values: 0:MulticastUDP,1:SocketCAN
    // @User: Advanced
    AP_GROUPINFO("CAN_TYPE1", 30, SIM,  can_transport[0], uint8_t(CANTransport::MulticastUDP)),
#endif

#if HAL_NUM_CAN_IFACES > 1
    // @Param: CAN_TYPE2
    // @DisplayName: transport type for second CAN interface
    // @Description: transport type for second CAN interface
    // @Values: 0:MulticastUDP,1:SocketCAN
    // @User: Advanced
    AP_GROUPINFO("CAN_TYPE2", 31, SIM,  can_transport[1], uint8_t(CANTransport::MulticastUDP)),
#endif

    AP_GROUPINFO("SONAR_SCALE",   32, SIM,  sonar_scale, 12.1212f),
    // @Param: FLOW_ENABLE
    // @DisplayName: Opflow Enable
    // @Description: Enable simulated Optical Flow sensor
    // @Values: 0:Disable,1:Enabled
    AP_GROUPINFO("FLOW_ENABLE",   33, SIM,  flow_enable, 0),
    AP_GROUPINFO("TERRAIN",       34, SIM,  terrain_enable, 1),
    // @Param: FLOW_RATE
    // @DisplayName: Opflow Rate
    // @Description: Opflow Data Rate
    // @Units: Hz
    AP_GROUPINFO("FLOW_RATE",     35, SIM,  flow_rate, 10),
    // @Param: FLOW_DELAY
    // @DisplayName: Opflow Delay
    // @Description: Opflow data delay
    // @Units: ms
    AP_GROUPINFO("FLOW_DELAY",    36, SIM,  flow_delay, 0),
    AP_GROUPINFO("ADSB_COUNT",    45, SIM,  adsb_plane_count, -1),
    AP_GROUPINFO("ADSB_RADIUS",   46, SIM,  adsb_radius_m, 10000),
    AP_GROUPINFO("ADSB_ALT",      47, SIM,  adsb_altitude_m, 1000),
    AP_GROUPINFO("PIN_MASK",      50, SIM,  pin_mask, 0),
    AP_GROUPINFO("ADSB_TX",       51, SIM,  adsb_tx, 0),
    // @Param: SPEEDUP
    // @DisplayName: Sim Speedup
    // @Description: Runs the simulation at multiples of normal speed. Do not use if realtime physics, like RealFlight, is being used
    // @Range: 1 10
    // @User: Advanced    
    AP_GROUPINFO("SPEEDUP",       52, SIM,  speedup, -1),
    // @Param: IMU_POS
    // @DisplayName: IMU Offsets
    // @Description: XYZ position of the IMU accelerometer relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("IMU_POS",       53, SIM,  imu_pos_offset, 0),
    AP_SUBGROUPEXTENSION("",      54, SIM,  var_ins),
    AP_GROUPINFO("SONAR_POS",     55, SIM,  rngfnd_pos_offset, 0),
    // @Param: FLOW_POS
    // @DisplayName: Opflow Pos
    // @Description: XYZ position of the optical flow sensor focal point relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("FLOW_POS",      56, SIM,  optflow_pos_offset, 0),
    AP_GROUPINFO("ENGINE_FAIL",   58, SIM,  engine_fail,  0),
    AP_SUBGROUPINFO(models, "",   59, SIM, SIM::ModelParm),
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
    // @Param: WIND_T
    // @DisplayName: Wind Profile Type
    // @Description: Selects how wind varies from surface to WIND_T_ALT
    // @Values: 0:square law,1: none, 2:linear-see WIND_T_COEF
    // @User: Advanced    
    AP_GROUPINFO("WIND_T"      ,15, SIM,  wind_type, SIM::WIND_TYPE_SQRT),
    // @Param: WIND_T_ALT
    // @DisplayName: Full Wind Altitude
    // @Description: Altitude at which wind reaches full strength, decaying from full strength as altitude lowers to ground level
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("WIND_T_ALT"  ,16, SIM,  wind_type_alt, 60),
    // @Param: WIND_T_COEF
    // @DisplayName: Linear Wind Curve Coeff
    // @Description: For linear wind profile,wind is reduced by (Altitude-WIND_T_ALT) x this value
    // @User: Advanced
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

    // @Param: WOW_PIN
    // @DisplayName: Weight on Wheels Pin
    // @Description: SITL set this simulated pin to true if vehicle is on ground
    // @User: Advanced
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
    // @Param: FLOW_RND
    // @DisplayName: Opflow noise
    // @Description: Optical Flow sensor measurement noise in rad/sec
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
    // @Param: OPOS_LAT
    // @DisplayName: Original Position (Latitude)
    // @Description: Specifies vehicle's startup latitude
    // @User: Advanced
    AP_GROUPINFO("OPOS_LAT",    51, SIM,  opos.lat, -35.363261f),
    // @Param: OPOS_LNG
    // @DisplayName: Original Position (Longitude)
    // @Description: Specifies vehicle's startup longitude
    // @User: Advanced
    AP_GROUPINFO("OPOS_LNG",    52, SIM,  opos.lng, 149.165230f),
    // @Param: OPOS_ALT
    // @DisplayName: Original Position (Altitude)
    // @Description: Specifies vehicle's startup altitude (AMSL)
    // @User: Advanced
    AP_GROUPINFO("OPOS_ALT",    53, SIM,  opos.alt, 584.0f),
    // @Param: OPOS_HDG
    // @DisplayName: Original Position (Heading)
    // @Description: Specifies vehicle's startup heading (0-360)
    // @User: Advanced
    AP_GROUPINFO("OPOS_HDG",    54, SIM,  opos.hdg, 353.0f),

    // extra delay per main loop
    AP_GROUPINFO("LOOP_DELAY",  55, SIM,  loop_delay, 0),

    // @Path: ./SIM_Buzzer.cpp
    AP_SUBGROUPINFO(buzzer_sim, "BZ_", 56, SIM, Buzzer),

    // @Path: ./SIM_ToneAlarm.cpp
    AP_SUBGROUPINFO(tonealarm_sim, "TA_", 57, SIM, ToneAlarm),

    AP_GROUPINFO("EFI_TYPE",    58, SIM,  efi_type,  SIM::EFI_TYPE_NONE),

    // 59 was SAFETY_STATE

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

    // @Param: VICON_POS_X
    // @DisplayName: SITL vicon position on vehicle in Forward direction
    // @Description: SITL vicon position on vehicle in Forward direction
    // @Units: m
    // @Range: 0 10
    // @User: Advanced

    // @Param: VICON_POS_Y
    // @DisplayName: SITL vicon position on vehicle in Right direction
    // @Description: SITL vicon position on vehicle in Right direction
    // @Units: m
    // @Range: 0 10
    // @User: Advanced

    // @Param: VICON_POS_Z
    // @DisplayName: SITL vicon position on vehicle in Down direction
    // @Description: SITL vicon position on vehicle in Down direction
    // @Units: m
    // @Range: 0 10
    // @User: Advanced    
    AP_GROUPINFO("VICON_POS",     14, SIM,  vicon_pos_offset, 0),

    // Buyoancy for submarines
    AP_GROUPINFO_FRAME("BUOYANCY", 15, SIM, buoyancy, 1, AP_PARAM_FRAME_SUB),

    // @Param: VICON_GLIT_X
    // @DisplayName: SITL vicon position glitch North
    // @Description: SITL vicon position glitch North
    // @Units: m
    // @User: Advanced

    // @Param: VICON_GLIT_Y
    // @DisplayName: SITL vicon position glitch East
    // @Description: SITL vicon position glitch East
    // @Units: m
    // @User: Advanced

    // @Param: VICON_GLIT_Z
    // @DisplayName: SITL vicon position glitch Down
    // @Description: SITL vicon position glitch Down
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("VICON_GLIT",    16, SIM,  vicon_glitch, 0),

    // @Param: VICON_FAIL
    // @DisplayName: SITL vicon failure
    // @Description: SITL vicon failure
    // @Values: 0:Vicon Healthy, 1:Vicon Failed
    // @User: Advanced
    AP_GROUPINFO("VICON_FAIL",    17, SIM,  vicon_fail, 0),

    // @Param: VICON_YAW
    // @DisplayName: SITL vicon yaw angle in earth frame
    // @Description: SITL vicon yaw angle in earth frame
    // @Units: deg
    // @Range: 0 360
    // @User: Advanced
    AP_GROUPINFO("VICON_YAW",     18, SIM,  vicon_yaw, 0),

    // @Param: VICON_YAWERR
    // @DisplayName: SITL vicon yaw error
    // @Description: SITL vicon yaw added to reported yaw sent to vehicle
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("VICON_YAWERR",  19, SIM,  vicon_yaw_error, 0),

    // @Param: VICON_TMASK
    // @DisplayName: SITL vicon type mask
    // @Description: SITL vicon messages sent
    // @Bitmask: 0:VISION_POSITION_ESTIMATE, 1:VISION_SPEED_ESTIMATE, 2:VICON_POSITION_ESTIMATE, 3:VISION_POSITION_DELTA, 4:ODOMETRY
    // @User: Advanced
    AP_GROUPINFO("VICON_TMASK",   20, SIM,  vicon_type_mask, 3),

    // @Param: VICON_VGLI_X
    // @DisplayName: SITL vicon velocity glitch North
    // @Description: SITL vicon velocity glitch North
    // @Units: m/s
    // @User: Advanced

    // @Param: VICON_VGLI_Y
    // @DisplayName: SITL vicon velocity glitch East
    // @Description: SITL vicon velocity glitch East
    // @Units: m/s
    // @User: Advanced

    // @Param: VICON_VGLI_Z
    // @DisplayName: SITL vicon velocity glitch Down
    // @Description: SITL vicon velocity glitch Down
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("VICON_VGLI",    21, SIM,  vicon_vel_glitch, 0),

    AP_GROUPINFO("RATE_HZ",  22, SIM,  loop_rate_hz, SIM_RATE_HZ_DEFAULT),

    // @Param: IMU_COUNT
    // @DisplayName: IMU count
    // @Description: Number of simulated IMUs to create
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

    // @Param: ESC_TELEM
    // @DisplayName: Simulated ESC Telemetry
    // @Description: enable perfect simulated ESC telemetry
    // @User: Advanced
    AP_GROUPINFO("ESC_TELEM", 40, SIM, esc_telem, 1),

    AP_GROUPINFO("ESC_ARM_RPM", 41, SIM,  esc_rpm_armed, 0.0f),

    // @Param: UART_LOSS
    // @DisplayName: UART byte loss percentage
    // @Description: Sets percentage of outgoing byte loss on UARTs
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO("UART_LOSS", 42, SIM,  uart_byte_loss_pct, 0),

    // @Group: ARSPD_
    // @Path: ./SITL_Airspeed.cpp
    AP_SUBGROUPINFO(airspeed[0], "ARSPD_", 50, SIM, AirspeedParm),
#if AIRSPEED_MAX_SENSORS > 1
    // @Group: ARSPD2_
    // @Path: ./SITL_Airspeed.cpp
    AP_SUBGROUPINFO(airspeed[1], "ARSPD2_", 51, SIM, AirspeedParm),
#endif

    // @Param: ADSB_TYPES
    // @DisplayName: Simulated ADSB Type mask
    // @Description: specifies which simulated ADSB types are active
    // @User: Advanced
    // @Bitmask: 0:MAVLink,3:SageTechMXS
    AP_GROUPINFO("ADSB_TYPES",    52, SIM,  adsb_types, 1),

#ifdef WITH_SITL_OSD
    // @Param: OSD_COLUMNS
    // @DisplayName: Simulated OSD number of text columns
    // @Description: Simulated OSD number of text columns
    // @Range: 10 100
    AP_GROUPINFO("OSD_COLUMNS",   53, SIM,  osd_columns, 30),

    // @Param: OSD_ROWS
    // @DisplayName: Simulated OSD number of text rows
    // @Description: Simulated OSD number of text rows
    // @Range: 10 100
    AP_GROUPINFO("OSD_ROWS",     54, SIM,  osd_rows, 16),
#endif

#ifdef SFML_JOYSTICK
    AP_SUBGROUPEXTENSION("",      63, SIM,  var_sfml_joystick),
#endif // SFML_JOYSTICK

    AP_GROUPEND
};

// user settable parameters for the barometers
const AP_Param::GroupInfo SIM::BaroParm::var_info[] = {
    AP_GROUPINFO("RND",      1, SIM::BaroParm,  noise, 0.2f),
    // @Param: BARO_DRIFT
    // @DisplayName: Baro altitude drift
    // @Description: Barometer altitude drifts at this rate
    // @Units: m/s
    // @User: Advanced
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
    AP_GROUPINFO("WCF_UP",  11, SIM::BaroParm, wcof_zp, 0.0),
    AP_GROUPINFO("WCF_DN",  12, SIM::BaroParm, wcof_zn, 0.0),
    AP_GROUPEND
};

#if HAL_SIM_GPS_ENABLED
// GPS SITL parameters
const AP_Param::GroupInfo SIM::var_gps[] = {
    // @Param: GPS_DISABLE
    // @DisplayName: GPS 1 disable
    // @Description: Disables GPS 1
    // @Values: 0:Enable, 1:GPS Disabled
    // @User: Advanced
    AP_GROUPINFO("GPS_DISABLE",    1, SIM,  gps_disable[0], 0),
    // @Param: GPS_LAG_MS
    // @DisplayName: GPS 1 Lag
    // @Description: GPS 1 lag
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("GPS_LAG_MS",     2, SIM,  gps_delay_ms[0], 100),
    // @Param: GPS_TYPE
    // @DisplayName: GPS 1 type
    // @Description: Sets the type of simulation used for GPS 1
    // @Values: 0:None, 1:UBlox, 5:NMEA, 6:SBP, 7:File, 8:Nova, 9:SBP2, 11:Trimble, 19:MSP
    // @User: Advanced
    AP_GROUPINFO("GPS_TYPE",       3, SIM,  gps_type[0],  GPS::Type::UBLOX),
    // @Param: GPS_BYTELOSS
    // @DisplayName: GPS Byteloss
    // @Description: Percent of bytes lost from GPS 1
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO("GPS_BYTELOSS",   4, SIM,  gps_byteloss[0],  0),
    // @Param: GPS_NUMSATS
    // @DisplayName: GPS 1 Num Satellites
    // @Description: Number of satellites GPS 1 has in view
    AP_GROUPINFO("GPS_NUMSATS",    5, SIM,  gps_numsats[0],   10),
    // @Param: GPS_GLITCH
    // @DisplayName: GPS 1 Glitch
    // @Description: Glitch offsets of simulated GPS 1 sensor
    // @Vector3Parameter: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_GLITCH",     6, SIM,  gps_glitch[0],  0),
    // @Param: GPS_HZ
    // @DisplayName: GPS 1 Hz
    // @Description: GPS 1 Update rate
    // @Units: Hz
    AP_GROUPINFO("GPS_HZ",         7, SIM,  gps_hertz[0],  5),
    // @Param: GPS_DRIFTALT
    // @DisplayName: GPS 1 Altitude Drift
    // @Description: GPS 1 altitude drift error
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GPS_DRIFTALT",   8, SIM,  gps_drift_alt[0], 0),
    // @Param: GPS_POS
    // @DisplayName: GPS 1 Position
    // @Description: GPS 1 antenna phase center position relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("GPS_POS",        9, SIM,  gps_pos_offset[0], 0),
    // @Param: GPS_NOISE
    // @DisplayName: GPS 1 Noise
    // @Description: Amplitude of the GPS1 altitude error
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GPS_NOISE",     10, SIM,  gps_noise[0], 0),
    // @Param: GPS_LOCKTIME
    // @DisplayName: GPS 1 Lock Time
    // @Description: Delay in seconds before GPS1 acquires lock
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("GPS_LOCKTIME",  11, SIM,  gps_lock_time[0], 0),
    // @Param: GPS_ALT_OFS
    // @DisplayName: GPS 1 Altitude Offset
    // @Description: GPS 1 Altitude Error
    // @Units: m
    AP_GROUPINFO("GPS_ALT_OFS",   12, SIM,  gps_alt_offset[0], 0),
    // @Param: GPS_HDG
    // @DisplayName: GPS 1 Heading
    // @Description: Enable GPS1 output of NMEA heading HDT sentence or UBLOX_RELPOSNED
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GPS_HDG",       13, SIM,  gps_hdg_enabled[0], SIM::GPS_HEADING_NONE),
    // @Param: GPS_ACC
    // @DisplayName: GPS 1 Accuracy
    // @Description: GPS 1 Accuracy
    // @User: Advanced
    AP_GROUPINFO("GPS_ACC",       14, SIM,  gps_accuracy[0], 0.3),
    // @Param: GPS_VERR
    // @DisplayName: GPS 1 Velocity Error
    // @Description: GPS 1 Velocity Error Offsets in NED
    // @Vector3Parameter: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_VERR",      15, SIM,  gps_vel_err[0], 0),
    // @Param: GPS_JAM
    // @DisplayName: GPS jamming enable
    // @Description: Enable simulated GPS jamming
    // @User: Advanced
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO("GPS_JAM",       16, SIM,  gps_jam[0], 0),
    // @Param: GPS2_DISABLE
    // @DisplayName: GPS 2 disable
    // @Description: Disables GPS 2
    // @Values: 0:Enable, 1:GPS Disabled
    // @User: Advanced
    AP_GROUPINFO("GPS2_DISABLE",  30, SIM,  gps_disable[1], 1),
    // @Param: GPS2_LAG_MS
    // @DisplayName: GPS 2 Lag
    // @Description: GPS 2 lag in ms
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("GPS2_LAG_MS",   31, SIM,  gps_delay_ms[1], 100),
    // @Param: GPS2_TYPE
    // @CopyFieldsFrom: SIM_GPS_TYPE
    // @DisplayName: GPS 2 type
    // @Description: Sets the type of simulation used for GPS 2
    AP_GROUPINFO("GPS2_TYPE",     32, SIM,  gps_type[1],  GPS::Type::UBLOX),
    // @Param: GPS2_BYTELOS
    // @DisplayName: GPS 2 Byteloss
    // @Description: Percent of bytes lost from GPS 2
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO("GPS2_BYTELOS",  33, SIM,  gps_byteloss[1],  0),
    // @Param: GPS2_NUMSATS
    // @DisplayName: GPS 2 Num Satellites
    // @Description: Number of satellites GPS 2 has in view
    AP_GROUPINFO("GPS2_NUMSATS",  34, SIM,  gps_numsats[1],   10),
    // @Param: GPS2_GLTCH
    // @DisplayName: GPS 2 Glitch
    // @Description: Glitch offsets of simulated GPS 2 sensor
    // @Vector3Parameter: 1
    // @User: Advanced
    AP_GROUPINFO("GPS2_GLTCH",    35, SIM,  gps_glitch[1],  0),
    // @Param: GPS2_HZ
    // @DisplayName: GPS 2 Hz
    // @Description: GPS 2 Update rate
    // @Units: Hz
    AP_GROUPINFO("GPS2_HZ",       36, SIM,  gps_hertz[1],  5),
    // @Param: GPS2_DRFTALT
    // @DisplayName: GPS 2 Altitude Drift
    // @Description: GPS 2 altitude drift error
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GPS2_DRFTALT",  37, SIM,  gps_drift_alt[1], 0),
    // @Param: GPS2_POS
    // @DisplayName: GPS 2 Position
    // @Description: GPS 2 antenna phase center position relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("GPS2_POS",      38, SIM,  gps_pos_offset[1], 0),
    // @Param: GPS2_NOISE
    // @DisplayName: GPS 2 Noise
    // @Description: Amplitude of the GPS2 altitude error
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GPS2_NOISE",    39, SIM,  gps_noise[1], 0),
    // @Param: GPS2_LCKTIME
    // @DisplayName: GPS 2 Lock Time
    // @Description: Delay in seconds before GPS2 acquires lock
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("GPS2_LCKTIME",  40, SIM,  gps_lock_time[1], 0),
    // @Param: GPS2_ALT_OFS
    // @DisplayName: GPS 2 Altitude Offset
    // @Description: GPS 2 Altitude Error
    // @Units: m
    AP_GROUPINFO("GPS2_ALT_OFS",  41, SIM,  gps_alt_offset[1], 0),
    // @Param: GPS2_HDG
    // @DisplayName: GPS 2 Heading
    // @Description: Enable GPS2 output of NMEA heading HDT sentence or UBLOX_RELPOSNED
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GPS2_HDG",      42, SIM,  gps_hdg_enabled[1], SIM::GPS_HEADING_NONE),
    // @Param: GPS2_ACC
    // @DisplayName: GPS 2 Accuracy
    // @Description: GPS 2 Accuracy
    // @User: Advanced
    AP_GROUPINFO("GPS2_ACC",      43, SIM,  gps_accuracy[1], 0.3),
    // @Param: GPS2_VERR
    // @DisplayName: GPS 2 Velocity Error
    // @Description: GPS 2 Velocity Error Offsets in NED
    // @Vector3Parameter: 1
    // @User: Advanced
    AP_GROUPINFO("GPS2_VERR",     44, SIM,  gps_vel_err[1], 0),

    // @Param: INIT_LAT_OFS
    // @DisplayName: Initial Latitude Offset
    // @Description: GPS initial lat offset from origin
    AP_GROUPINFO("INIT_LAT_OFS",  45, SIM,  gps_init_lat_ofs, 0),
    // @Param: INIT_LON_OFS
    // @DisplayName: Initial Longitude Offset
    // @Description: GPS initial lon offset from origin
    AP_GROUPINFO("INIT_LON_OFS",  46, SIM,  gps_init_lon_ofs, 0),
    // @Param: INIT_ALT_OFS
    // @DisplayName: Initial Altitude Offset
    // @Description: GPS initial alt offset from origin
    AP_GROUPINFO("INIT_ALT_OFS",  47, SIM,  gps_init_alt_ofs, 0),

    // @Param: GPS_LOG_NUM
    // @DisplayName: GPS Log Number
    // @Description: Log number for GPS:update_file()
    AP_GROUPINFO("GPS_LOG_NUM",   48, SIM,  gps_log_num, 0),

    // @Param: GPS2_JAM
    // @DisplayName: GPS jamming enable
    // @Description: Enable simulated GPS jamming
    // @User: Advanced
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO("GPS2_JAM",      49, SIM,  gps_jam[1], 0),

    AP_GROUPEND
};
#endif  // HAL_SIM_GPS_ENABLED

// Mag SITL parameters
const AP_Param::GroupInfo SIM::var_mag[] = {
    AP_GROUPINFO("MAG_RND",        1, SIM,  mag_noise,   0),
    AP_GROUPINFO("MAG_MOT",        2, SIM,  mag_mot, 0),
    AP_GROUPINFO("MAG_DELAY",      3, SIM,  mag_delay, 0),
    AP_GROUPINFO("MAG1_OFS",        4, SIM,  mag_ofs[0], 0),
    AP_GROUPINFO("MAG_ALY",        5, SIM,  mag_anomaly_ned, 0),
    AP_GROUPINFO("MAG_ALY_HGT",    6, SIM,  mag_anomaly_hgt, 1.0f),
    AP_GROUPINFO("MAG1_DIA",        7, SIM,  mag_diag[0], 0),
    AP_GROUPINFO("MAG1_ODI",        8, SIM,  mag_offdiag[0], 0),
    AP_GROUPINFO("MAG1_ORIENT",     9, SIM,  mag_orient[0], 0),
    AP_GROUPINFO("MAG1_SCALING",  10, SIM,  mag_scaling[0], 1),
    // @Param: MAG1_DEVID
    // @DisplayName: MAG1 Device ID
    // @Description: Device ID of simulated compass 1
    // @User: Advanced
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
    // @Param: MAG1_FAIL
    // @DisplayName: MAG1 Failure
    // @Description: Simulated failure of MAG1
    // @Values: 0:Disabled, 1:MAG1 Failure
    // @User: Advanced
    AP_GROUPINFO("MAG1_FAIL",     26, SIM,  mag_fail[0], 0),
#if HAL_COMPASS_MAX_SENSORS > 1
    AP_GROUPINFO("MAG2_OFS",      19, SIM,  mag_ofs[1], 0),
    AP_GROUPINFO("MAG2_DIA",      20, SIM,  mag_diag[1], 0),
    AP_GROUPINFO("MAG2_ODI",      21, SIM,  mag_offdiag[1], 0),
    AP_GROUPINFO("MAG2_ORIENT",   22, SIM,  mag_orient[1], 0),
    // @Param: MAG2_FAIL
    // @DisplayName: MAG2 Failure
    // @Description: Simulated failure of MAG2
    // @Values: 0:Disabled, 1:MAG2 Failure
    // @User: Advanced
    AP_GROUPINFO("MAG2_FAIL",     27, SIM,  mag_fail[1], 0),
    AP_GROUPINFO("MAG2_SCALING",  28, SIM,  mag_scaling[1], 1),
#endif
#if HAL_COMPASS_MAX_SENSORS > 2
    AP_GROUPINFO("MAG3_OFS",      23, SIM,  mag_ofs[2], 0),
    AP_GROUPINFO("MAG3_DIA",      24, SIM,  mag_diag[2], 0),
    AP_GROUPINFO("MAG3_ODI",      25, SIM,  mag_offdiag[2], 0),
    // @Param: MAG3_FAIL
    // @DisplayName: MAG3 Failure
    // @Description: Simulated failure of MAG3
    // @Values: 0:Disabled, 1:MAG3 Failure
    // @User: Advanced
    AP_GROUPINFO("MAG3_FAIL",     29, SIM,  mag_fail[2], 0),
    AP_GROUPINFO("MAG3_SCALING",  30, SIM,  mag_scaling[2], 1),
    AP_GROUPINFO("MAG3_ORIENT",   36, SIM,  mag_orient[2], 0),
#endif

    // @Param: MAG_SAVE_IDS
    // @DisplayName: Save MAG devids on startup
    // @Description: This forces saving of compass devids on startup so that simulated compasses start as calibrated
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("MAG_SAVE_IDS", 37, SIM, mag_save_ids, 1),

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
    // @Param: ACC1_BIAS
    // @DisplayName: Accel 1 bias
    // @Description: bias of simulated accelerometer sensor
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC1_BIAS",     5, SIM, accel_bias[0], 0),
#if INS_MAX_INSTANCES > 1
    // @Param: ACC2_BIAS
    // @DisplayName: Accel 2 bias
    // @CopyFieldsFrom: SIM_ACC1_BIAS
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC2_BIAS",     6, SIM, accel_bias[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: ACC3_BIAS
    // @DisplayName: Accel 3 bias
    // @CopyFieldsFrom: SIM_ACC1_BIAS
    // @Vector3Parameter: 1
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
    // @Param: GYR1_SCALE
    // @DisplayName: Gyro 1 scaling factor
    // @Description: scaling factors applied to simulated gyroscope
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR1_SCALE",   14, SIM, gyro_scale[0], 0),
#if INS_MAX_INSTANCES > 1
    // @Param: GYR2_SCALE
    // @DisplayName: Gyro 2 scaling factor
    // @CopyFieldsFrom: SIM_GYR1_SCALE
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR2_SCALE",   15, SIM, gyro_scale[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: GYR3_SCALE
    // @DisplayName: Gyro 3 scaling factor
    // @CopyFieldsFrom: SIM_GYR1_SCALE
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR3_SCALE",   16, SIM, gyro_scale[2], 0),
#endif
    // @Param: ACCEL1_FAIL
    // @DisplayName: ACCEL1 Failure
    // @Description: Simulated failure of ACCEL1
    // @Values: 0:Disabled, 1:ACCEL1 Failure
    // @User: Advanced
    AP_GROUPINFO("ACCEL1_FAIL",  17, SIM, accel_fail[0],  0),
#if INS_MAX_INSTANCES > 1
    // @Param: ACCEL2_FAIL
    // @DisplayName: ACCEL2 Failure
    // @Description: Simulated failure of ACCEL2
    // @Values: 0:Disabled, 1:ACCEL2 Failure
    // @User: Advanced
    AP_GROUPINFO("ACCEL2_FAIL",  18, SIM, accel_fail[1],  0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: ACCEL3_FAIL
    // @DisplayName: ACCEL3 Failure
    // @Description: Simulated failure of ACCEL3
    // @Values: 0:Disabled, 1:ACCEL3 Failure
    // @User: Advanced
    AP_GROUPINFO("ACCEL3_FAIL",  19, SIM, accel_fail[2],  0),
#endif
    // @Param: GYRO_FAIL_MSK
    // @DisplayName: Gyro Failure Mask
    // @Description: Determines if the gyro reading updates are stopped when for an IMU simulated failure by ACCELx_FAIL params
    // @Values: 0:Disabled, 1:Readings stopped
    // @User: Advanced
    AP_GROUPINFO("GYR_FAIL_MSK", 20, SIM, gyro_fail_mask,  0),
    // @Param: ACC_FAIL_MSK
    // @DisplayName: Accelerometer Failure Mask
    // @Description: Determines if the acclerometer reading updates are stopped when for an IMU simulated failure by ACCELx_FAIL params
    // @Values: 0:Disabled, 1:Readings stopped
    // @User: Advanced
    AP_GROUPINFO("ACC_FAIL_MSK", 21, SIM, accel_fail_mask,  0),

    // @Param: ACC1_SCAL
    // @DisplayName: Accel 1 scaling factor
    // @Description: scaling factors applied to simulated accelerometer
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC1_SCAL",    22, SIM, accel_scale[0], 0),
#if INS_MAX_INSTANCES > 1
    // @Param: ACC2_SCAL
    // @DisplayName: Accel 2 scaling factor
    // @CopyFieldsFrom: SIM_ACC1_SCAL
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC2_SCAL",    23, SIM, accel_scale[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: ACC3_SCAL
    // @DisplayName: Accel 3 scaling factor
    // @CopyFieldsFrom: SIM_ACC1_SCAL
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC3_SCAL",    24, SIM, accel_scale[2], 0),
#endif
    AP_GROUPINFO("ACC_TRIM",     25, SIM, accel_trim, 0),

#if APM_BUILD_TYPE(APM_BUILD_Rover)
    // @Param{Rover}: SAIL_TYPE
    // @DisplayName: Sailboat simulation sail type
    // @Description: 0: mainsail with sheet, 1: directly actuated wing
    AP_GROUPINFO("SAIL_TYPE",     26, SIM, sail_type, 0),
#endif

    // @Param: JSON_MASTER
    // @DisplayName: JSON master instance
    // @Description: the instance number to  take servos from
    AP_GROUPINFO("JSON_MASTER",     27, SIM, ride_along_master, 0),

    // @Param: OH_MASK
    // @DisplayName: SIM-on_hardware Output Enable Mask
    // @Description: channels which are passed through to actual hardware when running sim on actual hardware
    AP_GROUPINFO("OH_MASK",     28, SIM, on_hardware_output_enable_mask, 0),
#if AP_SIM_INS_FILE_ENABLED
    // read and write IMU data to/from files
    AP_GROUPINFO("GYR_FILE_RW", 29, SIM, gyro_file_rw, INSFileMode::INS_FILE_NONE),
    AP_GROUPINFO("ACC_FILE_RW", 30, SIM, accel_file_rw, INSFileMode::INS_FILE_NONE),
#endif

    // @Param: GYR1_BIAS_X
    // @DisplayName: First Gyro bias on X axis
    // @Description: First Gyro bias on X axis
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR1_BIAS_Y
    // @DisplayName: First Gyro bias on Y axis
    // @Description: First Gyro bias on Y axis
    // @Units: rad/s
    // @User: Advanced

    // @Param: GYR1_BIAS_Z
    // @DisplayName: First Gyro bias on Z axis
    // @Description: First Gyro bias on Z axis
    // @Units: rad/s
    // @User: Advanced


    AP_GROUPINFO("GYR1_BIAS",     31, SIM, gyro_bias[0], 0),
#if INS_MAX_INSTANCES > 1
    // @Param: GYR2_BIAS_X
    // @CopyFieldsFrom: SIM_GYR1_BIAS_X
    // @DisplayName: Second Gyro bias on X axis
    // @Description: Second Gyro bias on X axis

    // @Param: GYR2_BIAS_Y
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Y
    // @DisplayName: Second Gyro bias on Y axis
    // @Description: Second Gyro bias on Y axis

    // @Param: GYR2_BIAS_Z
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Z
    // @DisplayName: Second Gyro bias on Z axis
    // @Description: Second Gyro bias on Z axis
    AP_GROUPINFO("GYR2_BIAS",     32, SIM, gyro_bias[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: GYR3_BIAS_X
    // @CopyFieldsFrom: SIM_GYR1_BIAS_X
    // @DisplayName: Third Gyro bias on X axis
    // @Description: Third Gyro bias on X axis

    // @Param: GYR3_BIAS_Y
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Y
    // @DisplayName: Third Gyro bias on Y axis
    // @Description: Third Gyro bias on Y axis

    // @Param: GYR3_BIAS_Z
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Z
    // @DisplayName: Third Gyro bias on Z axis
    // @Description: Third Gyro bias on Z axis

    AP_GROUPINFO("GYR3_BIAS",     33, SIM, gyro_bias[2], 0),
#endif

#if INS_MAX_INSTANCES > 3
    // @Param: ACC4_SCAL
    // @DisplayName: Accel 4 scaling factor
    // @CopyFieldsFrom: SIM_ACC1_SCAL
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC4_SCAL",    34, SIM, accel_scale[3], 0),

    // @Param: ACCEL4_FAIL
    // @DisplayName: ACCEL4 Failure
    // @Description: Simulated failure of ACCEL4
    // @Values: 0:Disabled, 1:ACCEL4 Failure
    // @User: Advanced
    AP_GROUPINFO("ACCEL4_FAIL",  35, SIM, accel_fail[3],  0),

    // @Param: GYR4_SCALE
    // @DisplayName: Gyro 4 scaling factor
    // @CopyFieldsFrom: SIM_GYR1_SCALE
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR4_SCALE",   36, SIM, gyro_scale[3], 0),

    AP_GROUPINFO("ACC4_RND",     37, SIM, accel_noise[3], 0),

    AP_GROUPINFO("GYR4_RND",     38, SIM, gyro_noise[3],  0),

    // @Param: ACC4_BIAS
    // @DisplayName: Accel 4 bias
    // @CopyFieldsFrom: SIM_ACC1_BIAS
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC4_BIAS",    39, SIM, accel_bias[3], 0),

    // @Param: GYR4_BIAS_X
    // @CopyFieldsFrom: SIM_GYR1_BIAS_X
    // @DisplayName: Fourth Gyro bias on X axis
    // @Description: Fourth Gyro bias on X axis

    // @Param: GYR4_BIAS_Y
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Y
    // @DisplayName: Fourth Gyro bias on Y axis
    // @Description: Fourth Gyro bias on Y axis

    // @Param: GYR4_BIAS_Z
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Z
    // @DisplayName: Fourth Gyro bias on Z axis
    // @Description: Fourth Gyro bias on Z axis

    AP_GROUPINFO("GYR4_BIAS",    40, SIM, gyro_bias[3], 0),

#endif

#if INS_MAX_INSTANCES > 4
    // @Param: ACC5_SCAL
    // @DisplayName: Accel 4 scaling factor
    // @CopyFieldsFrom: SIM_ACC1_SCAL
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC5_SCAL",    41, SIM, accel_scale[4], 0),


    // @Param: ACCEL5_FAIL
    // @DisplayName: ACCEL5 Failure
    // @Description: Simulated failure of ACCEL5
    // @Values: 0:Disabled, 1:ACCEL5 Failure
    // @User: Advanced
    AP_GROUPINFO("ACCEL5_FAIL",  42, SIM, accel_fail[4],  0),

    // @Param: GYR5_SCALE
    // @DisplayName: Gyro 5 scaling factor
    // @CopyFieldsFrom: SIM_GYR1_SCALE
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR5_SCALE",   43, SIM, gyro_scale[4], 0),

    AP_GROUPINFO("ACC5_RND",     44, SIM, accel_noise[4], 0),

    AP_GROUPINFO("GYR5_RND",     45, SIM, gyro_noise[4],  0),

    // @Param: ACC5_BIAS
    // @DisplayName: Accel 5 bias
    // @CopyFieldsFrom: SIM_ACC1_BIAS
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC5_BIAS",    46, SIM, accel_bias[4], 0),

    // @Param: GYR5_BIAS_X
    // @CopyFieldsFrom: SIM_GYR1_BIAS_X
    // @DisplayName: Fifth Gyro bias on X axis
    // @Description: Fifth Gyro bias on X axis

    // @Param: GYR5_BIAS_Y
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Y
    // @DisplayName: Fifth Gyro bias on Y axis
    // @Description: Fifth Gyro bias on Y axis

    // @Param: GYR5_BIAS_Z
    // @CopyFieldsFrom: SIM_GYR1_BIAS_Z
    // @DisplayName: Fifth Gyro bias on Z axis
    // @Description: Fifth Gyro bias on Z axis

    AP_GROUPINFO("GYR5_BIAS",    47, SIM, gyro_bias[4], 0),
#endif

    // @Param: OH_RELAY_MSK
    // @DisplayName: SIM-on_hardware Relay Enable Mask
    // @Description: Allow relay output operation when running SIM-on-hardware
    AP_GROUPINFO("OH_RELAY_MSK",     48, SIM, on_hardware_relay_enable_mask, SIM_DEFAULT_ENABLED_RELAY_CHANNELS),

    // @Param: CLAMP_CH
    // @DisplayName: Simulated Clamp Channel
    // @Description: If non-zero the vehicle will be clamped in position until the value on this servo channel passes 1800PWM
    AP_GROUPINFO("CLAMP_CH",     49, SIM, clamp_ch, 0),

    // the IMUT parameters must be last due to the enable parameters
#if HAL_INS_TEMPERATURE_CAL_ENABLE
    AP_SUBGROUPINFO(imu_tcal[0], "IMUT1_", 61, SIM, AP_InertialSensor_TCal),
#if INS_MAX_INSTANCES > 1
    AP_SUBGROUPINFO(imu_tcal[1], "IMUT2_", 62, SIM, AP_InertialSensor_TCal),
#endif
#if INS_MAX_INSTANCES > 2
    AP_SUBGROUPINFO(imu_tcal[2], "IMUT3_", 63, SIM, AP_InertialSensor_TCal),
#endif
#if INS_MAX_INSTANCES > 3
    AP_SUBGROUPINFO(imu_tcal[3], "IMUT4_", 60, SIM, AP_InertialSensor_TCal),
#endif
#if INS_MAX_INSTANCES > 4
    AP_SUBGROUPINFO(imu_tcal[4], "IMUT5_", 59, SIM, AP_InertialSensor_TCal),
#endif
#endif  // HAL_INS_TEMPERATURE_CAL_ENABLE
    AP_GROUPEND
};

// user settable parameters for the physics models
const AP_Param::GroupInfo SIM::ModelParm::var_info[] = {

#if AP_SIM_SHIP_ENABLED
    // @Group: SHIP_
    // @Path: ./SIM_Ship.cpp
    AP_SUBGROUPINFO(shipsim, "SHIP_", 1, SIM::ModelParm, ShipSim),
#endif
#if AP_SIM_STRATOBLIMP_ENABLED
    // @Group: SB_
    // @Path: ./SIM_StratoBlimp.cpp
    AP_SUBGROUPPTR(stratoblimp_ptr, "SB_",  2, SIM::ModelParm, StratoBlimp),
#endif

#if AP_SIM_GLIDER_ENABLED
    // @Group: GLD_
    // @Path: ./SIM_Glider.cpp
    AP_SUBGROUPPTR(glider_ptr, "GLD_",  3, SIM::ModelParm, Glider),
#endif

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
    if (stop_MAVLink_sim_state) {
        // Sim only MAVLink messages disabled to give more relaistic data rates
        return;
    }

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
    if (stop_MAVLink_sim_state) {
        // Sim only MAVLink messages disabled to give more relaistic data rates
        return;
    }

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
            state.speedD,
	        (int32_t)(state.latitude*1.0e7),
            (int32_t)(state.longitude*1.0e7));
}

#if HAL_LOGGING_ENABLED
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
#endif

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
    return nanf("");
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
