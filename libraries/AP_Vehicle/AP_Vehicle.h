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
#pragma once

/*
  this header holds a parameter structure for each vehicle type for
  parameters needed by multiple libraries
 */

#include "ModeReason.h" // reasons can't be defined in this header due to circular loops

#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>     // board configuration library
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Button/AP_Button.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Generator/AP_Generator.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>                    // Notify library
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Relay/AP_Relay.h>                      // APM relay
#include <AP_RSSI/AP_RSSI.h>                        // RSSI Library
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>      // Serial manager library
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_Hott_Telem/AP_Hott_Telem.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_Frsky_Telem/AP_Frsky_Parameters.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_VideoTX/AP_SmartAudio.h>

class AP_Vehicle : public AP_HAL::HAL::Callbacks {

public:

    AP_Vehicle() {
        if (_singleton) {
            AP_HAL::panic("Too many Vehicles");
        }
        AP_Param::setup_object_defaults(this, var_info);
        _singleton = this;
    }

    /* Do not allow copies */
    AP_Vehicle(const AP_Vehicle &other) = delete;
    AP_Vehicle &operator=(const AP_Vehicle&) = delete;

    static AP_Vehicle *get_singleton();

    // setup() is called once during vehicle startup to initialise the
    // vehicle object and the objects it contains.  The
    // AP_HAL_MAIN_CALLBACKS pragma creates a main(...) function
    // referencing an object containing setup() and loop() functions.
    // A vehicle is not expected to override setup(), but
    // subclass-specific initialisation can be done in init_ardupilot
    // which is called from setup().
    void setup(void) override final;

    // HAL::Callbacks implementation.
    void loop() override final;

    // set_mode *must* set control_mode_reason
    virtual bool set_mode(const uint8_t new_mode, const ModeReason reason) = 0;
    virtual uint8_t get_mode() const = 0;

    ModeReason get_control_mode_reason() const {
        return control_mode_reason;
    }

    /*
      common parameters for fixed wing aircraft
     */
    struct FixedWing {
        AP_Int8 throttle_min;
        AP_Int8 throttle_max;
        AP_Int8 throttle_slewrate;
        AP_Int8 throttle_cruise;
        AP_Int8 takeoff_throttle_max;
        AP_Int16 airspeed_min;
        AP_Int16 airspeed_max;
        AP_Int32 airspeed_cruise_cm;
        AP_Int32 min_gndspeed_cm;
        AP_Int8  crash_detection_enable;
        AP_Int16 roll_limit_cd;
        AP_Int16 pitch_limit_max_cd;
        AP_Int16 pitch_limit_min_cd;
        AP_Int8  autotune_level;
        AP_Int8  stall_prevention;
        AP_Int16 loiter_radius;

        struct Rangefinder_State {
            bool in_range:1;
            bool have_initial_reading:1;
            bool in_use:1;
            float initial_range;
            float correction;
            float initial_correction;
            float last_stable_correction;
            uint32_t last_correction_time_ms;
            uint8_t in_range_count;
            float height_estimate;
            float last_distance;
        };


        // stages of flight
        enum FlightStage {
            FLIGHT_TAKEOFF       = 1,
            FLIGHT_VTOL          = 2,
            FLIGHT_NORMAL        = 3,
            FLIGHT_LAND          = 4,
            FLIGHT_ABORT_LAND    = 7
        };
    };

    /*
      common parameters for multicopters
     */
    struct MultiCopter {
        AP_Int16 angle_max;
    };

    void get_common_scheduler_tasks(const AP_Scheduler::Task*& tasks, uint8_t& num_tasks);
    // implementations *MUST* fill in all passed-in fields or we get
    // Valgrind errors
    virtual void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) = 0;

    /*
      set the "likely flying" flag. This is not guaranteed to be
      accurate, but is the vehicle codes best guess as to the whether
      the vehicle is currently flying
    */
    void set_likely_flying(bool b) {
        if (b && !likely_flying) {
            _last_flying_ms = AP_HAL::millis();
        }
        likely_flying = b;
    }

    /*
      get the likely flying status. Returns true if the vehicle code
      thinks we are flying at the moment. Not guaranteed to be
      accurate
    */
    bool get_likely_flying(void) const {
        return likely_flying;
    }

    /*
      return time in milliseconds since likely_flying was set
      true. Returns zero if likely_flying is currently false
    */
    uint32_t get_time_flying_ms(void) const {
        if (!likely_flying) {
            return 0;
        }
        return AP_HAL::millis() - _last_flying_ms;
    }

    // returns true if the vehicle has crashed
    virtual bool is_crashed() const;

#ifdef ENABLE_SCRIPTING
    /*
      methods to control vehicle for use by scripting
    */
    virtual bool start_takeoff(float alt) { return false; }
    virtual bool set_target_location(const Location& target_loc) { return false; }
    virtual bool set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt) { return false; }
    virtual bool set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel) { return false; }
    virtual bool set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative) { return false; }
    virtual bool set_target_velocity_NED(const Vector3f& vel_ned) { return false; }
    virtual bool set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative) { return false; }
    virtual bool set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs) { return false; }

    // get target location (for use by scripting)
    virtual bool get_target_location(Location& target_loc) { return false; }

    // circle mode controls (only used by scripting with Copter)
    virtual bool get_circle_radius(float &radius_m) { return false; }
    virtual bool set_circle_rate(float rate_dps) { return false; }

    // set steering and throttle (-1 to +1) (for use by scripting with Rover)
    virtual bool set_steering_and_throttle(float steering, float throttle) { return false; }
#endif // ENABLE_SCRIPTING


    // control outputs enumeration
    enum class ControlOutput {
        Roll = 1,
        Pitch = 2,
        Throttle = 3,
        Yaw = 4,
        Lateral = 5,
        MainSail = 6,
        WingSail = 7,
        Walking_Height = 8,
        Last_ControlOutput  // place new values before this
    };

    // get control output (for use in scripting)
    // returns true on success and control_value is set to a value in the range -1 to +1
    virtual bool get_control_output(AP_Vehicle::ControlOutput control_output, float &control_value) { return false; }

    // write out harmonic notch log messages
    void write_notch_log_messages() const;
    // update the harmonic notch
    virtual void update_dynamic_notch() {};

    // zeroing the RC outputs can prevent unwanted motor movement:
    virtual bool should_zero_rc_outputs_on_reboot() const { return false; }

    // reboot the vehicle in an orderly manner, doing various cleanups
    // and flashing LEDs as appropriate
    void reboot(bool hold_in_bootloader);

    /*
      get the distance to next wp in meters
      return false if failed or n/a
     */
    virtual bool get_wp_distance_m(float &distance) const { return false; }

    /*
      get the current wp bearing in degrees
      return false if failed or n/a
     */
    virtual bool get_wp_bearing_deg(float &bearing) const { return false; }

    /*
      get the current wp crosstrack error in meters
      return false if failed or n/a
     */
    virtual bool get_wp_crosstrack_error_m(float &xtrack_error) const { return false; }

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    AP_Frsky_Parameters frsky_parameters;
#endif

protected:

    virtual void init_ardupilot() = 0;
    virtual void load_parameters() = 0;
    virtual void set_control_channels() {}

    // board specific config
    AP_BoardConfig BoardConfig;

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // board specific config for CAN bus
    AP_CANManager can_mgr;
#endif

    // main loop scheduler
    AP_Scheduler scheduler{FUNCTOR_BIND_MEMBER(&AP_Vehicle::fast_loop, void)};
    virtual void fast_loop();

    // IMU variables
    // Integration time; time last loop took to run
    float G_Dt;

    // sensor drivers
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    AP_Button button;
    RangeFinder rangefinder;

    AP_RSSI rssi;
#if HAL_RUNCAM_ENABLED
    AP_RunCam runcam;
#endif
#if HAL_GYROFFT_ENABLED
    AP_GyroFFT gyro_fft;
#endif
    AP_VideoTX vtx;
    AP_SerialManager serial_manager;

    AP_Relay relay;

    AP_ServoRelayEvents ServoRelayEvents;

    // notification object for LEDs, buzzers etc (parameter set to
    // false disables external leds)
    AP_Notify notify;

    // Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    AP_AHRS_NavEKF ahrs;
#else
    AP_AHRS_DCM ahrs;
#endif

#if HAL_HOTT_TELEM_ENABLED
    AP_Hott_Telem hott_telem;
#endif

#if HAL_VISUALODOM_ENABLED
    AP_VisualOdom visual_odom;
#endif

#if HAL_WITH_ESC_TELEM
    AP_ESC_Telem esc_telem;
#endif

#if HAL_MSP_ENABLED
    AP_MSP msp;
#endif

#if GENERATOR_ENABLED
    AP_Generator generator;
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    AP_ExternalAHRS externalAHRS;
#endif
    
#if HAL_SMARTAUDIO_ENABLED
    AP_SmartAudio smartaudio;
#endif

    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Scheduler::Task scheduler_tasks[];

#if OSD_ENABLED
    void publish_osd_info();
#endif

    ModeReason control_mode_reason = ModeReason::UNKNOWN;

private:

    // delay() callback that processing MAVLink packets
    static void scheduler_delay_callback();

    // if there's been a watchdog reset, notify the world via a
    // statustext:
    void send_watchdog_reset_statustext();

    // run notch update at either loop rate or 200Hz
    void update_dynamic_notch_at_specified_rate();

    bool likely_flying;         // true if vehicle is probably flying
    uint32_t _last_flying_ms;   // time when likely_flying last went true
    uint32_t _last_notch_update_ms; // last time update_dynamic_notch() was run

    static AP_Vehicle *_singleton;

    bool done_safety_init;
};

namespace AP {
    AP_Vehicle *vehicle();
};

extern const AP_HAL::HAL& hal;

extern const AP_Param::Info vehicle_var_info[];

#include "AP_Vehicle_Type.h"
