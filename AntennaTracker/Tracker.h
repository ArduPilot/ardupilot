/*
   Lead developers: Matthew Ridley and Andrew Tridgell

   Please contribute your ideas! See http://dev.ardupilot.org for details

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

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_GPS/AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro/AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass/AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <Filter/Filter.h>                     // Filter library

#include <AP_SerialManager/AP_SerialManager.h>   // Serial manager library
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_Logger/AP_Logger.h>
#include <AC_PID/AC_PID.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Stats/AP_Stats.h>                      // statistics library
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Beacon/AP_Beacon.h>
#include <AP_Common/AP_FWVersion.h>

// Configuration
#include "config.h"
#include "defines.h"

#include "RC_Channel.h"
#include "Parameters.h"
#include "GCS_Mavlink.h"
#include "GCS_Tracker.h"

#ifdef ENABLE_SCRIPTING
#include <AP_Scripting/AP_Scripting.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

class Tracker : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Tracker;
    friend class GCS_Tracker;
    friend class Parameters;

    Tracker(void);

    static const AP_FWVersion fwver;

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    Parameters g;

    // main loop scheduler
    AP_Scheduler scheduler;

    uint32_t start_time_ms = 0;

    AP_Logger logger;

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    NavEKF2 EKF2{&ahrs, rangefinder};
    NavEKF3 EKF3{&ahrs, rangefinder};
    AP_AHRS_NavEKF ahrs{EKF2, EKF3};
#else
    AP_AHRS_DCM ahrs;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif
    
    /**
       antenna control channels
    */
    RC_Channels_Tracker rc_channels;
    SRV_Channels servo_channels;

    LowPassFilterFloat yaw_servo_out_filt;
    LowPassFilterFloat pitch_servo_out_filt;

    bool yaw_servo_out_filt_init = false;
    bool pitch_servo_out_filt_init = false;

    GCS_Tracker _gcs; // avoid using this; use gcs()
    GCS_Tracker &gcs() { return _gcs; }

    AP_Stats stats;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Tracker::handle_battery_failsafe, void, const char*, const int8_t),
                           nullptr};
    struct Location current_loc;

    enum ControlMode control_mode  = INITIALISING;

#ifdef ENABLE_SCRIPTING
    AP_Scripting scripting;
#endif

    // Vehicle state
    struct {
        bool location_valid;    // true if we have a valid location for the vehicle
        Location location;      // lat, long in degrees * 10^7; alt in meters * 100
        Location location_estimate; // lat, long in degrees * 10^7; alt in meters * 100
        uint32_t last_update_us;    // last position update in microseconds
        uint32_t last_update_ms;    // last position update in milliseconds
        Vector3f vel;           // the vehicle's velocity in m/s
        int32_t relative_alt;	// the vehicle's relative altitude in meters * 100
    } vehicle;

    // Navigation controller state
    struct {
        float bearing;                  // bearing to vehicle in centi-degrees
        float distance;                 // distance to vehicle in meters
        float pitch;                    // pitch to vehicle in degrees (positive means vehicle is above tracker, negative means below)
        float angle_error_pitch;        // angle error between target and current pitch in centi-degrees
        float angle_error_yaw;          // angle error between target and current yaw in centi-degrees
        float alt_difference_baro;      // altitude difference between tracker and vehicle in meters according to the barometer.  positive value means vehicle is above tracker
        float alt_difference_gps;       // altitude difference between tracker and vehicle in meters according to the gps.  positive value means vehicle is above tracker
        float altitude_offset;          // offset in meters which is added to tracker altitude to align altitude measurements with vehicle's barometer
        bool manual_control_yaw         : 1;// true if tracker yaw is under manual control
        bool manual_control_pitch       : 1;// true if tracker pitch is manually controlled
        bool need_altitude_calibration  : 1;// true if tracker altitude has not been determined (true after startup)
        bool scan_reverse_pitch         : 1;// controls direction of pitch movement in SCAN mode
        bool scan_reverse_yaw           : 1;// controls direction of yaw movement in SCAN mode
    } nav_status = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false, true, false, false};

    // setup the var_info table
    AP_Param param_loader{var_info};

    bool target_set = false;
    bool stationary = true; // are we using the start lat and log?

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    // true if the compass's initial location has been set
    bool compass_init_location;

    // AntennaTracker.cpp
    void one_second_loop();
    void ten_hz_logging_loop();
    void stats_update();

    // control_auto.cpp
    void update_auto(void);
    void calc_angle_error(float pitch, float yaw, bool direction_reversed);
    void convert_ef_to_bf(float pitch, float yaw, float& bf_pitch, float& bf_yaw);
    bool convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw);
    bool get_ef_yaw_direction();

    // control_manual.cpp
    void update_manual(void);

    // control_scan.cpp
    void update_scan(void);

    // control_servo_test.cpp
    bool servo_test_set_servo(uint8_t servo_num, uint16_t pwm);

    // GCS_Mavlink.cpp
    void send_nav_controller_output(mavlink_channel_t chan);

    // Log.cpp
    void Log_Write_Attitude();
    void Log_Write_Vehicle_Baro(float pressure, float altitude);
    void Log_Write_Vehicle_Pos(int32_t lat,int32_t lng,int32_t alt, const Vector3f& vel);
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);

    // Parameters.cpp
    void load_parameters(void);

    // radio.cpp
    void read_radio();

    // sensors.cpp
    void update_ahrs();
    void compass_save();
    void update_compass(void);
    void accel_cal_update(void);
    void update_GPS(void);
    void handle_battery_failsafe(const char* type_str, const int8_t action);

    // servos.cpp
    void init_servos();
    void update_pitch_servo(float pitch);
    void update_pitch_position_servo(void);
    void update_pitch_onoff_servo(float pitch);
    void update_pitch_cr_servo(float pitch);
    void update_yaw_servo(float yaw);
    void update_yaw_position_servo(void);
    void update_yaw_onoff_servo(float yaw);
    void update_yaw_cr_servo(float yaw);

    // system.cpp
    void init_tracker();
    bool get_home_eeprom(struct Location &loc);
    bool set_home_eeprom(const Location &temp) WARN_IF_UNUSED;
    bool set_home(const Location &temp) WARN_IF_UNUSED;
    void arm_servos();
    void disarm_servos();
    void prepare_servos();
    void set_mode(enum ControlMode mode, ModeReason reason);
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    bool should_log(uint32_t mask);
    bool start_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    void exit_mission_callback() { return; }
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }

    // tracking.cpp
    void update_vehicle_pos_estimate();
    void update_tracker_position();
    void update_bearing_and_distance();
    void update_tracking(void);
    void tracking_update_position(const mavlink_global_position_int_t &msg);
    void tracking_update_pressure(const mavlink_scaled_pressure_t &msg);
    void tracking_manual_control(const mavlink_manual_control_t &msg);
    void update_armed_disarmed();

    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Tracker::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Tracker::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Tracker::exit_mission_callback, void)};
public:
    void mavlink_delay_cb();
};

extern Tracker tracker;
