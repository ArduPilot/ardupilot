/*
   Lead developers: Matthew Ridley and Andrew Tridgell

   Please contribute your ideas! See https://ardupilot.org/dev for details

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
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <Filter/Filter.h>                     // Filter library

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>

#include <SRV_Channel/SRV_Channel.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Stats/AP_Stats.h>                      // statistics library
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library

// Configuration
#include "config.h"
#include "defines.h"

#include "RC_Channel.h"
#include "Parameters.h"
#include "GCS_Mavlink.h"
#include "GCS_Tracker.h"

#include "AP_Arming.h"

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "mode.h"

class Tracker : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Tracker;
    friend class GCS_Tracker;
    friend class Parameters;
    friend class ModeAuto;
    friend class ModeGuided;
    friend class Mode;

    Tracker(void);

    void arm_servos();
    void disarm_servos();

private:
    Parameters g;

    uint32_t start_time_ms = 0;

    AP_Logger logger;

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

    Mode *mode_from_mode_num(enum Mode::Number num);

    Mode *mode = &mode_initialising;

    ModeAuto mode_auto;
    ModeInitialising mode_initialising;
    ModeManual mode_manual;
    ModeGuided mode_guided;
    ModeScan mode_scan;
    ModeServoTest mode_servotest;
    ModeStop mode_stop;

#if AP_SCRIPTING_ENABLED
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
    struct NavStatus {
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
    } nav_status;

    // setup the var_info table
    AP_Param param_loader{var_info};

    bool target_set = false;
    bool stationary = true; // are we using the start lat and log?

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    // Tracker.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void one_second_loop();
    void ten_hz_logging_loop();
    void stats_update();

    // GCS_Mavlink.cpp
    void send_nav_controller_output(mavlink_channel_t chan);

    // Log.cpp
    void Log_Write_Attitude();
    void Log_Write_Vehicle_Baro(float pressure, float altitude);
    void Log_Write_Vehicle_Pos(int32_t lat,int32_t lng,int32_t alt, const Vector3f& vel);
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);

    // Parameters.cpp
    void load_parameters(void) override;

    // radio.cpp
    void read_radio();

    // sensors.cpp
    void update_ahrs();
    void compass_save();
    void update_compass(void);
    void update_GPS(void);
    void handle_battery_failsafe(const char* type_str, const int8_t action);

    // servos.cpp
    void init_servos();
    void update_pitch_servo(float pitch);
    void update_pitch_position_servo(void);
    void update_pitch_onoff_servo(float pitch) const;
    void update_pitch_cr_servo(float pitch);
    void update_yaw_servo(float yaw);
    void update_yaw_position_servo(void);
    void update_yaw_onoff_servo(float yaw) const;
    void update_yaw_cr_servo(float yaw);

    // system.cpp
    void init_ardupilot() override;
    bool get_home_eeprom(struct Location &loc) const;
    bool set_home_eeprom(const Location &temp) WARN_IF_UNUSED;
    bool set_home(const Location &temp) WARN_IF_UNUSED;
    void prepare_servos();
    void set_mode(Mode &newmode, ModeReason reason);
    bool set_mode(uint8_t new_mode, ModeReason reason) override;
    uint8_t get_mode() const override { return (uint8_t)mode->number(); }
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
    void update_armed_disarmed() const;
    bool get_pan_tilt_norm(float &pan_norm, float &tilt_norm) const override;

    // Arming/Disarming management class
    AP_Arming_Tracker arming;

    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Tracker::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Tracker::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Tracker::exit_mission_callback, void)};
};

extern Tracker tracker;
