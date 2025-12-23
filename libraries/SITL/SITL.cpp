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
#include "SIM_FlightAxis.h"

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

    // @Param: DRIFT_SPEED
    // @DisplayName: Gyro drift speed
    // @Description: Gyro drift rate of change in degrees/second/minute
    AP_GROUPINFO("DRIFT_SPEED",    5, SIM,  drift_speed, 0.05f),
    // @Param: DRIFT_TIME
    // @DisplayName: Gyro drift time
    // @Description: Gyro drift duration of one full drift cycle (period in minutes)
    AP_GROUPINFO("DRIFT_TIME",     6, SIM,  drift_time,  5),
    // @Param: ENGINE_MUL
    // @DisplayName: Engine failure thrust scaler
    // @Description: Thrust from Motors in SIM_ENGINE_FAIL will be multiplied by this factor
    // @Range: 0 1
    AP_GROUPINFO("ENGINE_MUL",     8, SIM,  engine_mul,  0),
    // @Param: WIND_SPD
    // @DisplayName: Simulated Wind speed
    // @Description: Allows you to emulate wind in sim
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("WIND_SPD",       9, SIM,  wind_speed,  0),
    // @Param: WIND_DIR
    // @DisplayName: Direction simulated wind is coming from
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

    // @Param: SONAR_ROT
    // @DisplayName: Sonar rotation
    // @Description: Sonar rotation from rotations enumeration
    AP_GROUPINFO("SONAR_ROT",     17, SIM,  sonar_rot, Rotation::ROTATION_PITCH_270),
    // @Param: BATT_VOLTAGE
    // @DisplayName: Simulated battery voltage
    // @Description: Simulated battery voltage. Constant voltage when SIM_BATT_CAP_AH is 0, otherwise changing this parameter will re-initialize the state of charge of the battery based on this voltage versus the battery's maximum voltage (default is max voltage).
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("BATT_VOLTAGE",  19, SIM,  batt_voltage,  12.6f),
    // @Param: BATT_CAP_AH
    // @DisplayName: Simulated battery capacity
    // @Description: Simulated battery capacity. Set to 0 for unlimited capacity. Changing this parameter will re-initialize the state of charge of the battery.
    // @Units: Ah
    // @User: Advanced
    AP_GROUPINFO("BATT_CAP_AH",   20, SIM,  batt_capacity_ah,  0),
    // @Param: SONAR_GLITCH
    // @DisplayName: Sonar glitch probablility
    // @Description: Probablility a sonar glitch would happen
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("SONAR_GLITCH",  23, SIM,  sonar_glitch, 0),
    // @Param: SONAR_RND
    // @DisplayName: Sonar noise factor
    // @Description: Scaling factor for simulated sonar noise
    // @User: Advanced
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
    // @Values: 0:None,1:MulticastUDP,2:SocketCAN
    // @User: Advanced
    AP_GROUPINFO("CAN_TYPE1", 30, SIM,  can_transport[0], uint8_t(CANTransport::MulticastUDP)),
#endif

#if HAL_NUM_CAN_IFACES > 1
    // @Param: CAN_TYPE2
    // @DisplayName: transport type for second CAN interface
    // @Description: transport type for second CAN interface
    // @Values: 0:None,1:MulticastUDP,2:SocketCAN
    // @User: Advanced
    AP_GROUPINFO("CAN_TYPE2", 31, SIM,  can_transport[1], uint8_t(CANTransport::MulticastUDP)),
#endif

    // @Param: SONAR_SCALE
    // @DisplayName: Sonar conversion scale
    // @Description: Sonar conversion scale from distance to voltage
    // @Units: m/V
    AP_GROUPINFO("SONAR_SCALE",   32, SIM,  sonar_scale, 12.1212f),
    // @Param: FLOW_ENABLE
    // @DisplayName: Opflow Enable
    // @Description: Enable simulated Optical Flow sensor
    // @Values: 0:Disable,1:Enabled
    AP_GROUPINFO("FLOW_ENABLE",   33, SIM,  flow_enable, 0),
    // @Param: TERRAIN
    // @DisplayName: Terrain Enable
    // @Description: Enable using terrain for height
    // @Values: 0:Disable,1:Enabled
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
    // @Param: ADSB_COUNT
    // @DisplayName: Number of ADSB aircrafts
    // @Description: Total number of ADSB simulated aircraft
    AP_GROUPINFO("ADSB_COUNT",    45, SIM,  adsb_plane_count, -1),
    // @Param: ADSB_RADIUS
    // @DisplayName: ADSB radius stddev of another aircraft
    // @Description: Simulated standard deviation of radius in ADSB of another aircraft
    // @Units: m
    AP_GROUPINFO("ADSB_RADIUS",   46, SIM,  adsb_radius_m, 10000),
    // @Param: ADSB_ALT
    // @DisplayName: ADSB altitude of another aircraft
    // @Description: Simulated ADSB altitude of another aircraft
    // @Units: m
    AP_GROUPINFO("ADSB_ALT",      47, SIM,  adsb_altitude_m, 1000),
    // @Param: PIN_MASK
    // @DisplayName: GPIO emulation
    // @Description: SITL GPIO emulation
    AP_GROUPINFO("PIN_MASK",      50, SIM,  pin_mask, 0),
    // @Param: ADSB_TX
    // @DisplayName: ADSB transmit enable
    // @Description: ADSB transceiever enable and disable
    // @Values: 0:Transceiever disable, 1:Transceiever enable
    AP_GROUPINFO("ADSB_TX",       51, SIM,  adsb_tx, 0),
    // @Param: SPEEDUP
    // @DisplayName: Sim Speedup
    // @Description: Runs the simulation at multiples of normal speed. Do not use if realtime physics, like RealFlight, is being used
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("SPEEDUP",       52, SIM,  speedup, 1),
    // @Param: IMU_POS
    // @DisplayName: IMU Offsets
    // @Description: XYZ position of the IMU accelerometer relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("IMU_POS",       53, SIM,  imu_pos_offset, 0),
    AP_SUBGROUPEXTENSION("",      54, SIM,  var_ins),
    // @Param: SONAR_POS
    // @DisplayName: Sonar Offsets
    // @Description: XYZ position of the sonar relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("SONAR_POS",     55, SIM,  rngfnd_pos_offset, 0),
    // @Param: FLOW_POS
    // @DisplayName: Opflow Pos
    // @Description: XYZ position of the optical flow sensor focal point relative to the body frame origin
    // @Units: m
    // @Vector3Parameter: 1
    AP_GROUPINFO("FLOW_POS",      56, SIM,  optflow_pos_offset, 0),
    // @Param: ENGINE_FAIL
    // @DisplayName: Engine Fail Mask
    // @Description: mask of motors which SIM_ENGINE_MUL will be applied to
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32
    AP_GROUPINFO("ENGINE_FAIL",   58, SIM,  engine_fail,  0),
    AP_SUBGROUPINFO(models, "",   59, SIM, SIM::ModelParm),
    AP_SUBGROUPEXTENSION("",      60, SIM,  var_mag),
#if AP_SIM_GPS_ENABLED
    AP_SUBGROUPEXTENSION("",      61, SIM,  var_gps),
#endif
    AP_SUBGROUPEXTENSION("",      62, SIM,  var_info3),
    AP_SUBGROUPEXTENSION("",      63, SIM,  var_info2),
    AP_GROUPEND
};

// second table of user settable parameters for SITL. 
const AP_Param::GroupInfo SIM::var_info2[] = {
    // @Param: TEMP_START
    // @DisplayName: Start temperature
    // @Description: Baro start temperature
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("TEMP_START",   1, SIM,  temp_start,  25),
    // @Param: TEMP_BRD_OFF
    // @DisplayName: Baro temperature offset
    // @Description: Barometer board temperature offset from atmospheric temperature
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("TEMP_BRD_OFF", 2, SIM,  temp_board_offset, 20),
    // @Param: TEMP_TCONST
    // @DisplayName: Warmup time constant
    // @Description: Barometer warmup temperature time constant
    // @Units: degC
    // @User: Advanced
    AP_GROUPINFO("TEMP_TCONST",  3, SIM,  temp_tconst, 30),

    // @Param: TEMP_BFACTOR
    // @DisplayName: Baro temperature factor
    // @Description: A pressure change with temperature that closely matches what has been observed with a ICM-20789
    // @User: Advanced
    AP_GROUPINFO("TEMP_BFACTOR", 4, SIM,  temp_baro_factor, 0),
    // @Param: WIND_DIR_Z
    // @DisplayName: Simulated wind vertical direction
    // @Description: Allows you to set vertical wind direction (true deg) in sim. 0 means pure horizontal wind. 90 means pure updraft.
    // @Units: deg
    // @User: Advanced
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
    // @Param: RC_CHANCOUNT
    // @DisplayName: RC channel count
    // @Description: SITL RC channel count
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

    // @Param: VIB_FREQ
    // @DisplayName: Vibration frequency
    // @Description: Frequency of vibration applied to IMU readings in SITL
    // @Units: Hz
    // @Vector3Parameter: 1
    AP_GROUPINFO("VIB_FREQ",   26, SIM,  vibe_freq, 0),

    // @Group: PARA_
    // @Path: ./SIM_Parachute.cpp
    AP_SUBGROUPINFO(parachute_sim, "PARA_", 27, SIM, Parachute),

    // @Param: BAUDLIMIT_EN
    // @DisplayName: Telemetry bandwidth limitting
    // @Description: SITL enable bandwidth limitting on telemetry ports with non-zero values
    AP_GROUPINFO("BAUDLIMIT_EN",   28, SIM,  telem_baudlimit_enable, 0),

    // @Group: PLD_
    // @Path: ./SIM_Precland.cpp
    AP_SUBGROUPINFO(precland_sim, "PLD_", 29, SIM, SIM_Precland),

    // @Param: SHOVE_X
    // @DisplayName: Acceleration of shove x
    // @Description: Acceleration of shove to vehicle in x axis
    // @Units: m/s/s
    AP_GROUPINFO("SHOVE_X",     30, SIM,  shove.x, 0),
    // @Param: SHOVE_Y
    // @DisplayName: Acceleration of shove y
    // @Description: Acceleration of shove to vehicle in y axis
    // @Units: m/s/s
    AP_GROUPINFO("SHOVE_Y",     31, SIM,  shove.y, 0),
    // @Param: SHOVE_Z
    // @DisplayName: Acceleration of shove z
    // @Description: Acceleration of shove to vehicle in z axis
    // @Units: m/s/s
    AP_GROUPINFO("SHOVE_Z",     32, SIM,  shove.z, 0),
    // @Param: SHOVE_TIME
    // @DisplayName: Time length for shove
    // @Description: Force to the vehicle over a period of time
    // @Units: ms
    AP_GROUPINFO("SHOVE_TIME",  33, SIM,  shove.t, 0),
    
    // @Param: FLOW_RND
    // @DisplayName: Opflow noise
    // @Description: Optical Flow sensor measurement noise
    // @Units: rad/s
    AP_GROUPINFO("FLOW_RND",   34, SIM,  flow_noise,  0.05f),

    // @Param: TWIST_X
    // @DisplayName: Twist x
    // @Description: Rotational acceleration of twist x axis
    // @Units: rad/s/s
    AP_GROUPINFO("TWIST_X",     37, SIM,  twist.x, 0),
    // @Param: TWIST_Y
    // @DisplayName: Twist y
    // @Description: Rotational acceleration of twist y axis
    // @Units: rad/s/s
    AP_GROUPINFO("TWIST_Y",     38, SIM,  twist.y, 0),
    // @Param: TWIST_Z
    // @DisplayName: Twist z
    // @Description: Rotational acceleration of twist z axis
    // @Units: rad/s/s
    AP_GROUPINFO("TWIST_Z",     39, SIM,  twist.z, 0),
    // @Param: TWIST_TIME
    // @DisplayName: Twist time
    // @Description: Time that twist is applied on the vehicle
    // @Units: ms
    AP_GROUPINFO("TWIST_TIME",  40, SIM,  twist.t, 0),

    // @Param: GND_BEHAV
    // @DisplayName: Ground behavior
    // @Description: Ground behavior of aircraft (tailsitter, no movement, forward only)
    AP_GROUPINFO("GND_BEHAV",   41, SIM,  gnd_behav, -1),

    // @Param: IMU_ORIENT
    // @CopyFieldsFrom: AHRS_ORIENTATION
    // @DisplayName: IMU orientation
    // @Description: Simulated orientation of the IMUs
    AP_GROUPINFO("IMU_ORIENT",   42, SIM,  imu_orientation, 0),
    
    // sailboat wave and tide simulation parameters

    // @Param: WAVE_ENABLE
    // @DisplayName: Wave enable
    // @Description: Wave enable and modes
    // @Values: 0:disabled, 1: roll and pitch, 2: roll and pitch and heave
    AP_GROUPINFO("WAVE_ENABLE", 44, SIM,  wave.enable, 0.0f),
    // @Param: WAVE_LENGTH
    // @DisplayName: Wave length
    // @Description: Wave length in SITL
    // @Units: m
    AP_GROUPINFO("WAVE_LENGTH", 45, SIM,  wave.length, 10.0f),
    // @Param: WAVE_AMP
    // @DisplayName: Wave amplitude
    // @Description: Wave amplitude in SITL
    // @Units: m
    AP_GROUPINFO("WAVE_AMP",    46, SIM,  wave.amp, 0.5f),
    // @Param: WAVE_DIR
    // @DisplayName: Wave direction
    // @Description: Direction wave is coming from
    // @Units: deg
    AP_GROUPINFO("WAVE_DIR",    47, SIM,  wave.direction, 0.0f),
    // @Param: WAVE_SPEED
    // @DisplayName: Wave speed
    // @Description: Wave speed in SITL
    // @Units: m/s
    AP_GROUPINFO("WAVE_SPEED",  48, SIM,  wave.speed, 0.5f),
    // @Param: TIDE_DIR
    // @DisplayName: Tide direction
    // @Description: Tide direction wave is coming from
    // @Units: deg
    AP_GROUPINFO("TIDE_DIR",    49, SIM,  tide.direction, 0.0f),
    // @Param: TIDE_SPEED
    // @DisplayName: Tide speed
    // @Description: Tide speed in simulation
    // @Units: m/s
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
    // @Param: LOOP_DELAY
    // @DisplayName: Extra delay per main loop
    // @Description: Extra time delay per main loop
    // @Units: us
    AP_GROUPINFO("LOOP_DELAY",  55, SIM,  loop_delay, 0),

    // @Group: BZ_
    // @Path: ./SIM_Buzzer.cpp
    AP_SUBGROUPINFO(buzzer_sim, "BZ_", 56, SIM, Buzzer),

    // @Group: TA_
    // @Path: ./SIM_ToneAlarm.cpp
    AP_SUBGROUPINFO(tonealarm_sim, "TA_", 57, SIM, ToneAlarm),

    // @Param: EFI_TYPE
    // @DisplayName: Type of Electronic Fuel Injection
    // @Description: Different types of Electronic Fuel Injection (EFI) systems
    // @Values: 0:None,1:MegaSquirt EFI system, 2:Löweheiser EFI system, 8:Hirth engines
    AP_GROUPINFO("EFI_TYPE",    58, SIM,  efi_type,  SIM::EFI_TYPE_NONE),

    // 59 was SAFETY_STATE

    // @Param: VIB_MOT_HMNC
    // @DisplayName: Motor harmonics
    // @Description: Motor harmonics generated in SITL
    AP_GROUPINFO("VIB_MOT_HMNC", 60, SIM,  vibe_motor_harmonics, 1),
    // @Param: VIB_MOT_MASK
    // @DisplayName: Motor mask
    // @Description: Motor mask, allowing external simulators to mark motors
    AP_GROUPINFO("VIB_MOT_MASK", 5, SIM,  vibe_motor_mask, 0),
    // @Param: VIB_MOT_MAX
    // @DisplayName: Max motor vibration frequency
    // @Description: Max frequency to use as baseline for adding motor noise for the gyros and accels
    // @Units: Hz
    AP_GROUPINFO("VIB_MOT_MAX", 61, SIM,  vibe_motor, 0.0f),
    // @Param: INS_THR_MIN
    // @DisplayName: Minimum throttle INS noise
    // @Description: Minimum throttle for simulated ins noise
    AP_GROUPINFO("INS_THR_MIN", 62, SIM,  ins_noise_throttle_min, 0.1f),
    // @Param: VIB_MOT_MULT
    // @DisplayName: Vibration motor scale
    // @Description: Amplitude scaling of motor noise relative to gyro/accel noise
    AP_GROUPINFO("VIB_MOT_MULT", 63, SIM,  vibe_motor_scale, 1.0f),


    AP_GROUPEND

};

// third table of user settable parameters for SITL. 
const AP_Param::GroupInfo SIM::var_info3[] = {
    // @Param: ODOM_ENABLE
    // @DisplayName: Odometry enable
    // @Description: SITL odometry enabl
    // @Values: 0:Disable, 1:Enable
    AP_GROUPINFO("ODOM_ENABLE",   1, SIM,  odom_enable, 0),

    // @Param: LED_LAYOUT
    // @DisplayName: LED layout
    // @Description: LED layout config value
    AP_GROUPINFO("LED_LAYOUT",    11, SIM, led_layout, 0),

    // @Param: THML_SCENARI
    // @DisplayName: Thermal scenarios
    // @Description: Scenario for thermalling simulation, for soaring
    AP_GROUPINFO("THML_SCENARI",  12, SIM,  thermal_scenario, 0),

    // @Param{Sub}: BUOYANCY
    // @DisplayName: Buoyancy
    // @Description: Buyoancy for submarines
    AP_GROUPINFO_FRAME("BUOYANCY", 15, SIM, buoyancy, 1, AP_PARAM_FRAME_SUB),

    // @Param: RATE_HZ
    // @DisplayName: Loop rate
    // @Description: SITL Loop rate
    // @Units: Hz
    AP_GROUPINFO("RATE_HZ",  22, SIM,  loop_rate_hz, SIM_RATE_HZ_DEFAULT),

    // @Param: IMU_COUNT
    // @DisplayName: IMU count
    // @Description: Number of simulated IMUs to create
    AP_GROUPINFO("IMU_COUNT",    23, SIM,  imu_count,  2),

    // @Group: FTOWESC_
    // @Path: ./SIM_FETtecOneWireESC.cpp
    AP_SUBGROUPINFO(fetteconewireesc_sim, "FTOWESC_", 30, SIM, FETtecOneWireESC),

    // @Group: RICH_
    // @Path: ./SIM_RichenPower.cpp
    AP_SUBGROUPINFO(richenpower_sim, "RICH_", 31, SIM, RichenPower),

    // @Group: IE24_
    // @Path: ./SIM_IntelligentEnergy24.cpp
    AP_SUBGROUPINFO(ie24_sim, "IE24_", 32, SIM, IntelligentEnergy24),

    // user settable barometer parameters

    // @Param: BARO_COUNT
    // @DisplayName: Baro count
    // @Description: Number of simulated baros to create in SITL
    // @Range: 0 3
    AP_GROUPINFO("BARO_COUNT",    33, SIM,  baro_count, 2),

    // @Group: BARO_
    // @Path: ./SITL_Baro.cpp
    AP_SUBGROUPINFO(baro[0], "BARO_", 34, SIM, BaroParm),
#if BARO_MAX_INSTANCES > 1
    // @Group: BAR2_
    // @Path: ./SITL_Baro.cpp
    AP_SUBGROUPINFO(baro[1], "BAR2_", 35, SIM, BaroParm),
#endif
#if BARO_MAX_INSTANCES > 2
    // @Group: BAR3_
    // @Path: ./SITL_Baro.cpp
    AP_SUBGROUPINFO(baro[2], "BAR3_", 36, SIM, BaroParm),
#endif

    // @Param: TIME_JITTER
    // @DisplayName: Loop time jitter
    // @Description: Upper limit of random jitter in loop time
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("TIME_JITTER",  37, SIM,  loop_time_jitter_us, 0),

    // @Param: ESC_TELEM
    // @DisplayName: Simulated ESC Telemetry
    // @Description: enable perfect simulated ESC telemetry
    // @User: Advanced
    AP_GROUPINFO("ESC_TELEM", 40, SIM, esc_telem, 1),

    // @Param: ESC_ARM_RPM
    // @DisplayName: ESC RPM when armed
    // @Description: Simulated RPM when motors are armed
    // @User: Advanced
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

#if AP_SIM_VOLZ_ENABLED
    // @Group: VOLZ_
    // @Path: ./SIM_Volz.cpp
    AP_SUBGROUPINFO(volz_sim, "VOLZ_", 55, SIM, Volz),
#endif  //  AP_SIM_VOLZ_ENABLED

#if AP_SIM_VICON_ENABLED
    // @Group: VICON_
    // @Path: ./SIM_Vicon.cpp
    AP_SUBGROUPINFO(vicon, "VICON_", 56, SIM, ViconParms),
#endif  // AP_SIM_VICON_ENABLED

#ifdef SFML_JOYSTICK
    AP_SUBGROUPEXTENSION("",      63, SIM,  var_sfml_joystick),
#endif // SFML_JOYSTICK

    AP_GROUPEND
};


#if AP_SIM_GPS_ENABLED
// GPS SITL parameters
const AP_Param::GroupInfo SIM::var_gps[] = {
    //  1 was GPS_DISABLE
    //  2 was GPS_LAG_MS
    //  3 was GPS_TYPE
    //  4 was GPS_BYTELOSS
    //  5 was GPS_NUMSATS
    //  6 was GPS_GLITCH
    //  7 was GPS_HZ
    //  8 was GPS_DRIFTALT
    //  9 was GPS_POS
    // 10 was GPS_NOISE
    // 11 was GPS_LOCKTIME
    // 12 was GPS_ALT_OFS
    // 13 was GPS_HDG
    // 14 was GPS_ACC
    // 15 was GPS_VERR
    // 16 was GPS_JAM

    // 30 was GPS2_DISABLE
    // 31 was GPS2_LAG_MS
    // 32 was GPS2_TYPE
    // 33 was GPS2_BYTELOSS
    // 34 was GPS2_NUMSATS
    // 35 was GPS2_GLITCH
    // 36 was GPS2_HZ
    // 37 was GPS2_DRIFTALT
    // 38 was GPS2_POS
    // 39 was GPS2_NOISE
    // 40 was GPS2_LOCKTIME
    // 41 was GPS2_ALT_OFS
    // 42 was GPS2_HDG
    // 43 was GPS2_ACC
    // 44 was GPS2_VERR

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

    // 49 was GPS2_JAM

#if AP_SIM_MAX_GPS_SENSORS > 0
     // @Group: GPS1_
    // @Path: ./SIM_GPS.cpp
    AP_SUBGROUPINFO(gps[0], "GPS1_", 50, SIM, GPSParms),
#endif
#if AP_SIM_MAX_GPS_SENSORS > 1
    // @Group: GPS2_
    // @Path: ./SIM_GPS.cpp
    AP_SUBGROUPINFO(gps[1], "GPS2_", 51, SIM, GPSParms),
#endif
#if AP_SIM_MAX_GPS_SENSORS > 2
    // @Group: GPS3_
    // @Path: ./SIM_GPS.cpp
    AP_SUBGROUPINFO(gps[2], "GPS3_", 52, SIM, GPSParms),
#endif
#if AP_SIM_MAX_GPS_SENSORS > 3
    // @Group: GPS4_
    // @Path: ./SIM_GPS.cpp
    AP_SUBGROUPINFO(gps[3], "GPS4_", 53, SIM, GPSParms),
#endif

   AP_GROUPEND
};
#endif  // AP_SIM_GPS_ENABLED

// Mag SITL parameters
const AP_Param::GroupInfo SIM::var_mag[] = {
    // @Param: MAG_RND
    // @DisplayName: Mag motor noise factor
    // @Description: Scaling factor for simulated vibration from motors
    // @User: Advanced
    AP_GROUPINFO("MAG_RND",        1, SIM,  mag_noise,   0),
    // @Param: MAG_MOT
    // @DisplayName: Motor magnetic interference
    // @Description: Simulates distortion of magnetometer readings caused by motor current
    // @Units: mGauss/A
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("MAG_MOT",        2, SIM,  mag_mot, 0),
    // @Param: MAG_DELAY
    // @DisplayName: Mag measurement delay
    // @Description: Magnetometer measurement delay
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("MAG_DELAY",      3, SIM,  mag_delay, 0),
    // @Param: MAG1_OFS
    // @DisplayName: Magnetometer offset applied to SITL
    // @Description: Magnetometer offset injected into the simulation
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("MAG1_OFS",        4, SIM,  mag_ofs[0], 0),
    // @Param: MAG_ALY
    // @DisplayName: NED anomaly vector at ground level
    // @Description: Simulates localized magnetic field distortions at ground level that decays with altitude.
    // @Units: mGauss
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("MAG_ALY",        5, SIM,  mag_anomaly_ned, 0),
    // @Param: MAG_ALY_HGT
    // @DisplayName: Magnetic anomaly height
    // @Description: Height above ground where anomally strength has decayed to 1/8 of the ground level value
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("MAG_ALY_HGT",    6, SIM,  mag_anomaly_hgt, 1.0f),
    // @Param: MAG1_DIA_X
    // @DisplayName: Magnetometer soft-iron diagonal X component
    // @Description: DIA_X in the magnetometer soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: MAG1_DIA_Y
    // @DisplayName: Magnetometer soft-iron diagonal Y component
    // @Description: DIA_Y in the magnetometer soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: MAG1_DIA_Z
    // @DisplayName: Magnetometer soft-iron diagonal Z component
    // @Description: DIA_Z in the magnetometer soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("MAG1_DIA",        7, SIM,  mag_diag[0], 0),
    // @Param: MAG1_ODI_X
    // @DisplayName: Magnetometer soft-iron off-diagonal X component
    // @Description: ODI_X in the magnetometer soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: MAG1_ODI_Y
    // @DisplayName: Magnetometer soft-iron off-diagonal Y component
    // @Description: ODI_Y in the magnetometer soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: MAG1_ODI_Z
    // @DisplayName: Magnetometer soft-iron off-diagonal Z component
    // @Description: ODI_Z in the magnetometer soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("MAG1_ODI",        8, SIM,  mag_offdiag[0], 0),
    // @Param: MAG1_ORIENT
    // @DisplayName: MAG1 Orientation
    // @Description: MAG1 external compass orientation
    // @User: Advanced
    AP_GROUPINFO("MAG1_ORIENT",     9, SIM,  mag_orient[0], 0),
    // @Param: MAG1_SCALING
    // @DisplayName: MAG1 Scaling factor
    // @Description: Scale the compass 1 to simulate sensor scale factor errors
    // @User: Advanced
    AP_GROUPINFO("MAG1_SCALING",  10, SIM,  mag_scaling[0], 1),
    // @Param: MAG1_DEVID
    // @DisplayName: MAG1 Device ID
    // @Description: Device ID of simulated compass 1
    // @User: Advanced
    AP_GROUPINFO("MAG1_DEVID",    11, SIM,  mag_devid[0], 97539),
    // @Param: MAG2_DEVID
    // @DisplayName: MAG2 Device ID
    // @Description: Device ID of simulated compass 2
    // @User: Advanced
    AP_GROUPINFO("MAG2_DEVID",    12, SIM,  mag_devid[1], 131874),
#if MAX_CONNECTED_MAGS > 2
    // @Param: MAG3_DEVID
    // @DisplayName: MAG3 Device ID
    // @Description: Device ID of simulated compass 3
    // @User: Advanced
    AP_GROUPINFO("MAG3_DEVID",    13, SIM,  mag_devid[2], 263178),
#endif
#if MAX_CONNECTED_MAGS > 3
    // @Param: MAG4_DEVID
    // @DisplayName: MAG2 Device ID
    // @Description: Device ID of simulated compass 4
    // @User: Advanced
    AP_GROUPINFO("MAG4_DEVID",    14, SIM,  mag_devid[3], 97283),
#endif
#if MAX_CONNECTED_MAGS > 4
    // @Param: MAG5_DEVID
    // @DisplayName: MAG5 Device ID
    // @Description: Device ID of simulated compass 5
    // @User: Advanced
    AP_GROUPINFO("MAG5_DEVID",    15, SIM,  mag_devid[4], 97795),
#endif
#if MAX_CONNECTED_MAGS > 5
    // @Param: MAG6_DEVID
    // @DisplayName: MAG6 Device ID
    // @Description: Device ID of simulated compass 6
    // @User: Advanced
    AP_GROUPINFO("MAG6_DEVID",    16, SIM,  mag_devid[5], 98051),
#endif
#if MAX_CONNECTED_MAGS > 6
    // @Param: MAG7_DEVID
    // @DisplayName: MAG7 Device ID
    // @Description: Device ID of simulated compass 7
    // @User: Advanced
    AP_GROUPINFO("MAG7_DEVID",    17, SIM,  mag_devid[6], 0),
#endif
#if MAX_CONNECTED_MAGS > 7
    // @Param: MAG8_DEVID
    // @DisplayName: MAG8 Device ID
    // @Description: Device ID of simulated compass 8
    // @User: Advanced
    AP_GROUPINFO("MAG8_DEVID",    18, SIM,  mag_devid[7], 0),
#endif
    // @Param: MAG1_FAIL
    // @DisplayName: MAG1 Failure
    // @Description: Simulated failure of MAG1
    // @Values: 0:Disabled, 1:MAG1 Failure
    // @User: Advanced
    AP_GROUPINFO("MAG1_FAIL",     26, SIM,  mag_fail[0], 0),
#if HAL_COMPASS_MAX_SENSORS > 1

    // @Param: MAG2_OFS
    // @CopyFieldsFrom: SIM_MAG1_OFS
    // @Vector3Parameter: 1
    AP_GROUPINFO("MAG2_OFS",      19, SIM,  mag_ofs[1], 0),
    // @Param: MAG2_DIA_X
    // @CopyFieldsFrom: SIM_MAG1_DIA_X

    // @Param: MAG2_DIA_Y
    // @CopyFieldsFrom: SIM_MAG1_DIA_Y

    // @Param: MAG2_DIA_Z
    // @CopyFieldsFrom: SIM_MAG1_DIA_Z
    AP_GROUPINFO("MAG2_DIA",      20, SIM,  mag_diag[1], 0),
    // @Param: MAG2_ODI_X
    // @CopyFieldsFrom: SIM_MAG1_ODI_X

    // @Param: MAG2_ODI_Y
    // @CopyFieldsFrom: SIM_MAG1_ODI_Y

    // @Param: MAG2_ODI_Z
    // @CopyFieldsFrom: SIM_MAG1_ODI_Z
    AP_GROUPINFO("MAG2_ODI",      21, SIM,  mag_offdiag[1], 0),
    // @Param: MAG2_ORIENT
    // @DisplayName: MAG2 Orientation
    // @Description: MAG2 external compass orientation
    // @User: Advanced
    AP_GROUPINFO("MAG2_ORIENT",   22, SIM,  mag_orient[1], 0),
    // @Param: MAG2_FAIL
    // @DisplayName: MAG2 Failure
    // @Description: Simulated failure of MAG2
    // @Values: 0:Disabled, 1:MAG2 Failure
    // @User: Advanced
    AP_GROUPINFO("MAG2_FAIL",     27, SIM,  mag_fail[1], 0),
    // @Param: MAG2_SCALING
    // @DisplayName: MAG2 Scaling factor
    // @Description: Scale the compass 2 to simulate sensor scale factor errors
    // @User: Advanced
    AP_GROUPINFO("MAG2_SCALING",  28, SIM,  mag_scaling[1], 1),
#endif
#if HAL_COMPASS_MAX_SENSORS > 2

    // @Param: MAG3_OFS
    // @CopyFieldsFrom: SIM_MAG1_OFS
    // @Vector3Parameter: 1
    AP_GROUPINFO("MAG3_OFS",      23, SIM,  mag_ofs[2], 0),
    // @Param: MAG3_DIA_X
    // @CopyFieldsFrom: SIM_MAG1_DIA_X

    // @Param: MAG3_DIA_Y
    // @CopyFieldsFrom: SIM_MAG1_DIA_Y

    // @Param: MAG3_DIA_Z
    // @CopyFieldsFrom: SIM_MAG1_DIA_Z
    AP_GROUPINFO("MAG3_DIA",      24, SIM,  mag_diag[2], 0),
    // @Param: MAG3_ODI_X
    // @CopyFieldsFrom: SIM_MAG1_ODI_X

    // @Param: MAG3_ODI_Y
    // @CopyFieldsFrom: SIM_MAG1_ODI_Y

    // @Param: MAG3_ODI_Z
    // @CopyFieldsFrom: SIM_MAG1_ODI_Z
    AP_GROUPINFO("MAG3_ODI",      25, SIM,  mag_offdiag[2], 0),
    // @Param: MAG3_FAIL
    // @DisplayName: MAG3 Failure
    // @Description: Simulated failure of MAG3
    // @Values: 0:Disabled, 1:MAG3 Failure
    // @User: Advanced
    AP_GROUPINFO("MAG3_FAIL",     29, SIM,  mag_fail[2], 0),
    // @Param: MAG3_SCALING
    // @DisplayName: MAG3 Scaling factor
    // @Description: Scale the compass 3 to simulate sensor scale factor errors
    // @User: Advanced
    AP_GROUPINFO("MAG3_SCALING",  30, SIM,  mag_scaling[2], 1),
    // @Param: MAG3_ORIENT
    // @DisplayName: MAG3 Orientation
    // @Description: MAG3 external compass orientation
    // @User: Advanced
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
    // @Param: IMUT_START
    // @DisplayName: IMU temperature start
    // @Description: Starting IMU temperature of a curve
    AP_GROUPINFO("IMUT_START",    1, SIM, imu_temp_start,  25),
    // @Param: IMUT_END
    // @DisplayName: IMU temperature end
    // @Description: Ending IMU temperature of a curve
    AP_GROUPINFO("IMUT_END",      2, SIM, imu_temp_end, 45),
    // @Param: IMUT_TCONST
    // @DisplayName: IMU temperature time constant
    // @Description: IMU temperature time constant of the curve
    AP_GROUPINFO("IMUT_TCONST",   3, SIM, imu_temp_tconst, 300),
    // @Param: IMUT_FIXED
    // @DisplayName: IMU fixed temperature
    // @Description: IMU fixed temperature by user
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
    // @Param: GYR1_RND
    // @DisplayName: Gyro 1 motor noise factor
    // @Description: scaling factor for simulated vibration from motors
    // @User: Advanced
    AP_GROUPINFO("GYR1_RND",      8, SIM, gyro_noise[0],  0),
#if INS_MAX_INSTANCES > 1
    // @Param: GYR2_RND
    // @DisplayName: Gyro 2 motor noise factor
    // @CopyFieldsFrom: SIM_GYR1_RND
    AP_GROUPINFO("GYR2_RND",      9, SIM, gyro_noise[1],  0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: GYR3_RND
    // @DisplayName: Gyro 3 motor noise factor
    // @CopyFieldsFrom: SIM_GYR1_RND
    AP_GROUPINFO("GYR3_RND",     10, SIM, gyro_noise[2],  0),
#endif
    // @Param: ACC1_RND
    // @DisplayName: Accel 1 motor noise factor
    // @Description: scaling factor for simulated vibration from motors
    // @User: Advanced
    AP_GROUPINFO("ACC1_RND",     11, SIM, accel_noise[0], 0),
#if INS_MAX_INSTANCES > 1
    // @Param: ACC2_RND
    // @DisplayName: Accel 2 motor noise factor
    // @CopyFieldsFrom: SIM_ACC1_RND
    AP_GROUPINFO("ACC2_RND",     12, SIM, accel_noise[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: ACC3_RND
    // @DisplayName: Accel 3 motor noise factor
    // @CopyFieldsFrom: SIM_ACC1_RND
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
    // @Param: GYR_FAIL_MSK
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
    // @Param: ACC_TRIM
    // @DisplayName: Accelerometer trim
    // @Description: Trim applied to simulated accelerometer
    // @User: Advanced
    // @Vector3Parameter: 1
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

    // @Param: GYR_FILE_RW
    // @DisplayName: Gyro data to/from files
    // @Description: Read and write gyro data to/from files
    // @Values: 0:Stop writing data, 1:Read data from file, 2:Write data to a file, 3: Read data from file and stop on EOF
    AP_GROUPINFO("GYR_FILE_RW", 29, SIM, gyro_file_rw, INSFileMode::INS_FILE_NONE),
    // @Param: ACC_FILE_RW
    // @DisplayName: Accelerometer data to/from files
    // @Description: Read and write accelerometer data to/from files
    // @Values: 0:Stop writing data, 1:Read data from file, 2:Write data to a file, 3: Read data from file and stop on EOF
    AP_GROUPINFO("ACC_FILE_RW", 30, SIM, accel_file_rw, INSFileMode::INS_FILE_NONE),
#endif

    // @Param: GYR1_BIAS
    // @DisplayName: First Gyro bias
    // @Description: First Gyro bias
    // @Units: rad/s
    // @User: Advanced
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR1_BIAS",     31, SIM, gyro_bias[0], 0),
#if INS_MAX_INSTANCES > 1
    // @Param: GYR2_BIAS
    // @CopyFieldsFrom: SIM_GYR1_BIAS
    // @DisplayName: Second Gyro bias
    // @Description: Second Gyro bias
    // @Vector3Parameter: 1
    AP_GROUPINFO("GYR2_BIAS",     32, SIM, gyro_bias[1], 0),
#endif
#if INS_MAX_INSTANCES > 2
    // @Param: GYR3_BIAS
    // @CopyFieldsFrom: SIM_GYR1_BIAS
    // @DisplayName: Third Gyro bias
    // @Description: Third Gyro bias
    // @Vector3Parameter: 1
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

    // @Param: ACC4_RND
    // @DisplayName: Accel 4 motor noise factor
    // @CopyFieldsFrom: SIM_ACC1_RND
    AP_GROUPINFO("ACC4_RND",     37, SIM, accel_noise[3], 0),

    // @Param: GYR4_RND
    // @DisplayName: Gyro 4 motor noise factor
    // @CopyFieldsFrom: SIM_GYR1_RND
    AP_GROUPINFO("GYR4_RND",     38, SIM, gyro_noise[3],  0),

    // @Param: ACC4_BIAS
    // @DisplayName: Accel 4 bias
    // @CopyFieldsFrom: SIM_ACC1_BIAS
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC4_BIAS",    39, SIM, accel_bias[3], 0),

    // @Param: GYR4_BIAS
    // @CopyFieldsFrom: SIM_GYR1_BIAS
    // @DisplayName: Fourth Gyro bias
    // @Description: Fourth Gyro bias
    // @Vector3Parameter: 1
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

    // @Param: ACC5_RND
    // @DisplayName: Accel 5 motor noise factor
    // @CopyFieldsFrom: SIM_ACC1_RND
    AP_GROUPINFO("ACC5_RND",     44, SIM, accel_noise[4], 0),

    // @Param: GYR5_RND
    // @DisplayName: Gyro 5 motor noise factor
    // @CopyFieldsFrom: SIM_GYR1_RND
    AP_GROUPINFO("GYR5_RND",     45, SIM, gyro_noise[4],  0),

    // @Param: ACC5_BIAS
    // @DisplayName: Accel 5 bias
    // @CopyFieldsFrom: SIM_ACC1_BIAS
    // @Vector3Parameter: 1
    AP_GROUPINFO("ACC5_BIAS",    46, SIM, accel_bias[4], 0),

    // @Param: GYR5_BIAS
    // @CopyFieldsFrom: SIM_GYR1_BIAS
    // @DisplayName: Fifth Gyro bias
    // @Description: Fifth Gyro bias
    // @Vector3Parameter: 1
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

    // @Param: IMUT1_ENABLE
    // @DisplayName: Enable simulated temperature disturbance for sensor data
    // @Description: Enable the injection of temperature disturbance to the accelerometer and gyroscope data to simulate temperature calibration
    // @Values: 0:Disable,1:Enabled, 2: Learn Calibration
    // @User: Advanced

    // @Param: IMUT1_ACC1
    // @DisplayName: Applied simulated acceleration to accelerometer
    // @Description: This is the applied simulated acceleration to the 1st accelerometer
    // @User: Advanced
    // @Vector3Parameter: 1

    // @Param: IMUT1_ACC2
    // @DisplayName: Applied simulated acceleration to accelerometer
    // @Description: This is the applied simulated acceleration to the 2nd accelerometer
    // @User: Advanced
    // @Vector3Parameter: 1

    // @Param: IMUT1_ACC3
    // @DisplayName: Applied simulated acceleration to accelerometer
    // @Description: This is the applied simulated acceleration to the 3rd accelerometer
    // @User: Advanced
    // @Vector3Parameter: 1

    // @Param: IMUT1_GYR1
    // @DisplayName: Applied simulated angular rate to gyroscope
    // @Description: This is the applied simulated angular rate to the 1st gyroscope
    // @User: Advanced
    // @Vector3Parameter: 1

    // @Param: IMUT1_GYR2
    // @DisplayName: Applied simulated angular rate to gyroscope
    // @Description: This is the applied simulated angular rate to the 2nd gyroscope
    // @User: Advanced
    // @Vector3Parameter: 1

    // @Param: IMUT1_GYR3
    // @DisplayName: Applied simulated angular rate to gyroscope
    // @Description: This is the applied simulated angular rate to the 3rd gyroscope
    // @User: Advanced
    // @Vector3Parameter: 1

    // @Param: IMUT1_TMAX
    // @DisplayName: Simulated temperature calibration max
    // @Description: The maximum simulated temperature that the calibration is valid for. This must be at least 10 degrees above TMIN for calibration
    // @Units: degC
    // @Range: -70 80
    // @User: Advanced

    // @Param: IMUT1_TMIN
    // @DisplayName: Simulated temperature calibration min  
    // @Description: The minimum simulated temperature that the calibration is valid for
    // @Units: degC
    // @Range: -70 80
    // @User: Advanced
    AP_SUBGROUPINFO(imu_tcal[0], "IMUT1_", 61, SIM, AP_InertialSensor_TCal),
#if INS_MAX_INSTANCES > 1

    // @Param: IMUT2_ENABLE
    // @CopyFieldsFrom: SIM_IMUT1_ENABLE
    // @DisplayName: Enable simulated temperature disturbance for sensor data

    // @Param: IMUT2_ACC1
    // @CopyFieldsFrom: SIM_IMUT1_ACC1
    // @Vector3Parameter: 1

    // @Param: IMUT2_ACC2
    // @CopyFieldsFrom: SIM_IMUT1_ACC2
    // @Vector3Parameter: 1

    // @Param: IMUT2_ACC3
    // @CopyFieldsFrom: SIM_IMUT1_ACC3
    // @Vector3Parameter: 1

    // @Param: IMUT2_GYR1
    // @CopyFieldsFrom: SIM_IMUT1_GYR1
    // @Vector3Parameter: 1

    // @Param: IMUT2_GYR2
    // @CopyFieldsFrom: SIM_IMUT1_GYR2
    // @Vector3Parameter: 1

    // @Param: IMUT2_GYR3
    // @CopyFieldsFrom: SIM_IMUT1_GYR3
    // @Vector3Parameter: 1

    // @Param: IMUT2_TMAX
    // @CopyFieldsFrom: SIM_IMUT1_TMAX
    // @DisplayName: Simulated temperature calibration max

    // @Param: IMUT2_TMIN
    // @CopyFieldsFrom: SIM_IMUT1_TMIN
    // @DisplayName: Simulated temperature calibration min
    AP_SUBGROUPINFO(imu_tcal[1], "IMUT2_", 62, SIM, AP_InertialSensor_TCal),
#endif
#if INS_MAX_INSTANCES > 2

    // @Param: IMUT3_ENABLE
    // @CopyFieldsFrom: SIM_IMUT1_ENABLE
    // @DisplayName: Enable simulated temperature disturbance for sensor data

    // @Param: IMUT3_ACC1
    // @CopyFieldsFrom: SIM_IMUT1_ACC1
    // @Vector3Parameter: 1

    // @Param: IMUT3_ACC2
    // @CopyFieldsFrom: SIM_IMUT1_ACC2
    // @Vector3Parameter: 1

    // @Param: IMUT3_ACC3
    // @CopyFieldsFrom: SIM_IMUT1_ACC3
    // @Vector3Parameter: 1

    // @Param: IMUT3_GYR1
    // @CopyFieldsFrom: SIM_IMUT1_GYR1
    // @Vector3Parameter: 1

    // @Param: IMUT3_GYR2
    // @CopyFieldsFrom: SIM_IMUT1_GYR2
    // @Vector3Parameter: 1

    // @Param: IMUT3_GYR3
    // @CopyFieldsFrom: SIM_IMUT1_GYR3
    // @Vector3Parameter: 1

    // @Param: IMUT3_TMAX
    // @CopyFieldsFrom: SIM_IMUT1_TMAX
    // @DisplayName: Simulated temperature calibration max

    // @Param: IMUT3_TMIN
    // @CopyFieldsFrom: SIM_IMUT1_TMIN
    // @DisplayName: Simulated temperature calibration min
    AP_SUBGROUPINFO(imu_tcal[2], "IMUT3_", 63, SIM, AP_InertialSensor_TCal),
#endif
#if INS_MAX_INSTANCES > 3

    // @Param: IMUT4_ENABLE
    // @CopyFieldsFrom: SIM_IMUT1_ENABLE
    // @DisplayName: Enable simulated temperature disturbance for sensor data

    // @Param: IMUT4_ACC1
    // @CopyFieldsFrom: SIM_IMUT1_ACC1
    // @Vector3Parameter: 1

    // @Param: IMUT4_ACC2
    // @CopyFieldsFrom: SIM_IMUT1_ACC2
    // @Vector3Parameter: 1

    // @Param: IMUT4_ACC3
    // @CopyFieldsFrom: SIM_IMUT1_ACC3
    // @Vector3Parameter: 1

    // @Param: IMUT4_GYR1
    // @CopyFieldsFrom: SIM_IMUT1_GYR1
    // @Vector3Parameter: 1

    // @Param: IMUT4_GYR2
    // @CopyFieldsFrom: SIM_IMUT1_GYR2
    // @Vector3Parameter: 1

    // @Param: IMUT4_GYR3
    // @CopyFieldsFrom: SIM_IMUT1_GYR3
    // @Vector3Parameter: 1

    // @Param: IMUT4_TMAX
    // @CopyFieldsFrom: SIM_IMUT1_TMAX
    // @DisplayName: Simulated temperature calibration max

    // @Param: IMUT4_TMIN
    // @CopyFieldsFrom: SIM_IMUT1_TMIN
    // @DisplayName: Simulated temperature calibration min 
    AP_SUBGROUPINFO(imu_tcal[3], "IMUT4_", 60, SIM, AP_InertialSensor_TCal),
#endif
#if INS_MAX_INSTANCES > 4

    // @Param: IMUT5_ENABLE
    // @CopyFieldsFrom: SIM_IMUT1_ENABLE
    // @DisplayName: Enable simulated temperature disturbance for sensor data

    // @Param: IMUT5_ACC1
    // @CopyFieldsFrom: SIM_IMUT1_ACC1
    // @Vector3Parameter: 1

    // @Param: IMUT5_ACC2
    // @CopyFieldsFrom: SIM_IMUT1_ACC2
    // @Vector3Parameter: 1

    // @Param: IMUT5_ACC3
    // @CopyFieldsFrom: SIM_IMUT1_ACC3
    // @Vector3Parameter: 1

    // @Param: IMUT5_GYR1
    // @CopyFieldsFrom: SIM_IMUT1_GYR1
    // @Vector3Parameter: 1

    // @Param: IMUT5_GYR2
    // @CopyFieldsFrom: SIM_IMUT1_GYR2
    // @Vector3Parameter: 1

    // @Param: IMUT5_GYR3
    // @CopyFieldsFrom: SIM_IMUT1_GYR3
    // @Vector3Parameter: 1

    // @Param: IMUT5_TMAX
    // @CopyFieldsFrom: SIM_IMUT1_TMAX
    // @DisplayName: Simulated temperature calibration max

    // @Param: IMUT5_TMIN
    // @CopyFieldsFrom: SIM_IMUT1_TMIN
    // @DisplayName: Simulated temperature calibration min 
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

#if AP_SIM_SLUNGPAYLOAD_ENABLED
    // @Group: SLUP_
    // @Path: ./SIM_SlungPayload.cpp
    AP_SUBGROUPINFO(slung_payload_sim, "SLUP_", 4, SIM::ModelParm, SlungPayloadSim),
#endif

#if AP_SIM_FLIGHTAXIS_ENABLED
    // @Group: RFL_
    // @Path: ./SIM_FlightAxis.cpp
    AP_SUBGROUPPTR(flightaxis_ptr, "RFL_", 5, SIM::ModelParm, FlightAxis),
#endif

#if AP_SIM_TETHER_ENABLED
    // @Group: TETH_
    // @Path: ./SIM_Tether.cpp
    AP_SUBGROUPINFO(tether_sim, "TETH_", 6, SIM::ModelParm, TetherSim),
#endif

#if AP_SIM_AIS_ENABLED
    // @Group: AIS_
    // @Path: ./SIM_AIS.cpp
    AP_SUBGROUPPTR(ais_ptr, "AIS_", 7, SIM::ModelParm, AIS),
#endif  // AP_SIM_AIS_ENABLED

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
                              radians(state.rollDeg),
                              radians(state.pitchDeg),
                              radians(yaw),
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
            radians(state.rollDeg),
            radians(state.pitchDeg),
            radians(yaw),
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

    phi = radians(rollDeg);
    theta = radians(pitchDeg);
    phiDot = radians(rollRate);
    thetaDot = radians(pitchRate);
    psiDot = radians(yawRate);

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
    if (!location.get_vector_xy_from_origin_NE_cm(vehicle_pos_cm)) {
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
    if (!location2.get_vector_xy_from_origin_NE_cm(ray_endpos_cm)) {
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
                ::fprintf(postfile, "map circle %f %f %f blue\n", post_location.lat*1e-7, post_location.lng*1e-7, radius_cm*0.01);
            }
#endif
            Vector2f post_position_cm;
            if (!post_location.get_vector_xy_from_origin_NE_cm(post_position_cm)) {
                // should probably use SITL variables...
                min_dist_cm = 0;
                goto OUT;
            }
            Vector2f intersection_point_cm;
            if (Vector2f::circle_segment_intersection(ray_endpos_cm, vehicle_pos_cm, post_position_cm, radius_cm, intersection_point_cm)) {
                float dist_cm = (intersection_point_cm-vehicle_pos_cm).length();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                if (intersectionsfile != nullptr) {
                    Location intersection_point = location;
                    intersection_point.offset(intersection_point_cm.x*0.01,
                                              intersection_point_cm.y*0.01);
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

OUT:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (postfile != nullptr) {
        fclose(postfile);
    }
    if (intersectionsfile != nullptr) {
        fclose(intersectionsfile);
    }
#endif

    // ::fprintf(stderr, "Distance @%f = %fm\n", angle, min_dist_cm*0.01f);
    return min_dist_cm * 0.01f;
}

} // namespace SITL

namespace AP {

SITL::SIM *sitl()
{
    return SITL::SIM::get_singleton();
}

};

#endif // AP_SIM_ENABLED
