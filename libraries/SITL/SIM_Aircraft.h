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
  parent class for aircraft simulators
*/

#pragma once

#if AP_SIM_ENABLED

#include <AP_Math/AP_Math.h>

#include "SITL.h"
#include "SITL_Input.h"
#include "SIM_Sprayer.h"
#include "SIM_Gripper_Servo.h"
#include "SIM_Gripper_EPM.h"
#include "SIM_Parachute.h"
#include "SIM_Precland.h"
#include "SIM_RichenPower.h"
#include "SIM_Loweheiser.h"
#include "SIM_FETtecOneWireESC.h"
#include "SIM_I2C.h"
#include "SIM_Buzzer.h"
#include "SIM_Battery.h"
#include <Filter/Filter.h>
#include "SIM_JSON_Master.h"
#include "ServoModel.h"

namespace SITL {

/*
  parent class for all simulator types
 */
class Aircraft {
public:
    Aircraft(const char *frame_str);

    // called directly after constructor:
    virtual void set_start_location(const Location &start_loc, const float start_yaw);

    /*
      set simulation speedup
     */
    void set_speedup(float speedup);
    float get_speedup() const { return target_speedup; }

    /*
      set instance number
     */
    void set_instance(uint8_t _instance) {
        instance = _instance;
    }

    /*
      set directory for additional files such as aircraft models
     */
    void set_autotest_dir(const char *_autotest_dir) {
        autotest_dir = _autotest_dir;
    }

    /*  Create and set in/out socket for extenal simulator */
    virtual void set_interface_ports(const char* address, const int port_in, const int port_out) {};

    /*
      step the FDM by one time step
     */
    virtual void update(const struct sitl_input &input) = 0;

    void update_model(const struct sitl_input &input);

    void update_home();

    /* fill a sitl_fdm structure from the simulator state */
    void fill_fdm(struct sitl_fdm &fdm);

    /* smooth sensors to provide kinematic consistancy */
    void smooth_sensors(void);

    /* return normal distribution random numbers */
    static double rand_normal(double mean, double stddev);

    // get frame rate of model in Hz
    float get_rate_hz(void) const { return rate_hz; }

    const Vector3f &get_gyro(void) const {
        return gyro;
    }

    const Vector3f &get_velocity_ef(void) const {
        return velocity_ef;
    }

    // return TAS airspeed in earth frame
    const Vector3f &get_velocity_air_ef(void) const {
        return velocity_air_ef;
    }

    const Matrix3f &get_dcm(void) const {
        return dcm;
    }

    const Vector3f &get_mag_field_bf(void) const {
        return mag_bf;
    }

    float gross_mass() const { return mass + external_payload_mass; }

    virtual void set_config(const char* config) {
        config_ = config;
    }

    // return simulation origin:
    const Location &get_origin() const { return origin; }

    const Location &get_location() const { return location; }

    // get position relative to home
    Vector3d get_position_relhome() const;

    // get air density in kg/m^3
    float get_air_density(float alt_amsl) const;

    // distance the rangefinder is perceiving
    float rangefinder_range() const;

    void get_attitude(Quaternion &attitude) const {
        attitude.from_rotation_matrix(dcm);
    }

    const Location &get_home() const { return home; }
    float get_home_yaw() const { return home_yaw; }

    void set_buzzer(Buzzer *_buzzer) { buzzer = _buzzer; }
    void set_sprayer(Sprayer *_sprayer) { sprayer = _sprayer; }
    void set_parachute(Parachute *_parachute) { parachute = _parachute; }
    void set_richenpower(RichenPower *_richenpower) { richenpower = _richenpower; }
    void set_adsb(class ADSB *_adsb) { adsb = _adsb; }
#if AP_SIM_LOWEHEISER_ENABLED
    void set_loweheiser(Loweheiser *_loweheiser) { loweheiser = _loweheiser; }
#endif
    void set_fetteconewireesc(FETtecOneWireESC *_fetteconewireesc) { fetteconewireesc = _fetteconewireesc; }
    void set_ie24(IntelligentEnergy24 *_ie24) { ie24 = _ie24; }
    void set_gripper_servo(Gripper_Servo *_gripper) { gripper = _gripper; }
    void set_gripper_epm(Gripper_EPM *_gripper_epm) { gripper_epm = _gripper_epm; }
    void set_precland(SIM_Precland *_precland);
    void set_i2c(class I2C *_i2c) { i2c = _i2c; }
#if AP_TEST_DRONECAN_DRIVERS
    void set_dronecan_device(DroneCANDevice *_dronecan) { dronecan = _dronecan; }
#endif
    float get_battery_voltage() const { return battery_voltage; }
    float get_battery_temperature() const { return battery.get_temperature(); }

    ADSB *adsb;

protected:
    SIM *sitl;
    // origin of position vector
    Location origin;
    // home location
    Location home;
    bool home_is_set;
    Location location;

    float ground_level;
    float home_yaw;
    float frame_height;
    Matrix3f dcm;                        // rotation matrix, APM conventions, from body to earth
    Vector3f gyro;                       // rad/s
    Vector3f velocity_ef;                // m/s, earth frame
    Vector3f wind_ef;                    // m/s, earth frame
    Vector3f velocity_air_ef;            // velocity relative to airmass, earth frame (true airspeed)
    Vector3f velocity_air_bf;            // velocity relative to airmass, body frame
    Vector3d position;                   // meters, NED from origin
    float mass;                          // kg
    float external_payload_mass;         // kg
    Vector3f accel_body{0.0f, 0.0f, -GRAVITY_MSS}; // m/s/s NED, body frame
    float airspeed;                      // m/s, EAS airspeed
    float airspeed_pitot;                // m/s, EAS airspeed, as seen by fwd pitot tube
    float battery_voltage;
    float battery_current;
    float local_ground_level;            // ground level at local position
    bool lock_step_scheduled;
    uint32_t last_one_hz_ms;

    // battery model
    Battery battery;

    uint32_t motor_mask;
    float rpm[32];
    uint8_t rcin_chan_count;
    float rcin[12];

    virtual float rangefinder_beam_width() const { return 0; }
    virtual float perpendicular_distance_to_rangefinder_surface() const;

    struct {
        // data from simulated laser scanner, if available
        struct vector3f_array points;
        struct float_array ranges;
    } scanner;

    // Rangefinder
    float rangefinder_m[SITL_NUM_RANGEFINDERS];

    // Windvane apparent wind
    struct {
        float speed;
        float direction;
    } wind_vane_apparent;

    // Wind Turbulence simulated Data
    float turbulence_azimuth;
    float turbulence_horizontal_speed;  // m/s
    float turbulence_vertical_speed;    // m/s

    Vector3f mag_bf;  // local earth magnetic field vector in Gauss, earth frame

    uint64_t time_now_us;

    const float gyro_noise = radians(0.1f);
    const float accel_noise = 0.3f;
    float rate_hz = 1200.0f;
    float target_speedup;
    uint64_t frame_time_us;
    uint64_t last_wall_time_us;
    uint32_t last_fps_report_ms;
    float achieved_rate_hz;  // achieved speedup rate
    int64_t sleep_debt_us;
    uint32_t last_frame_count;
    uint8_t instance;
    const char *autotest_dir;
    const char *frame;
    bool use_time_sync = true;
    float last_speedup = -1.0f;
    const char *config_ = "";
    float eas2tas = 1.0;
    float air_density = SSL_AIR_DENSITY;

    // allow for AHRS_ORIENTATION
    AP_Int8 *ahrs_orientation;
    enum Rotation last_imu_rotation;
    AP_Float* custom_roll;
    AP_Float* custom_pitch;
    AP_Float* custom_yaw;

    enum GroundBehaviour {
        GROUND_BEHAVIOR_NONE = 0,
        GROUND_BEHAVIOR_NO_MOVEMENT,
        GROUND_BEHAVIOR_FWD_ONLY,
        GROUND_BEHAVIOR_TAILSITTER,
    } ground_behavior;

    bool use_smoothing;
    bool disable_origin_movement;

    float ground_height_difference() const;

    virtual bool on_ground() const;

    // returns height above ground level in metres
    float hagl() const;  // metres

    /* update location from position */
    void update_position(void);

    /* update body frame magnetic field */
    void update_mag_field_bf(void);

    /* advance time by deltat in seconds */
    void time_advance();

    /* setup the frame step time */
    void setup_frame_time(float rate, float speedup);

    /* adjust frame_time calculation */
    void adjust_frame_time(float rate);

    /* try to synchronise simulation time with wall clock time, taking
       into account desired speedup */
    void sync_frame_time(void);

    /* add noise based on throttle level (from 0..1) */
    void add_noise(float throttle);

    /* return a monotonic wall clock time in microseconds */
    uint64_t get_wall_time_us(void) const;

    // update attitude and relative position
    void update_dynamics(const Vector3f &rot_accel);

    // update wind vector
    void update_wind(const struct sitl_input &input);

    // return filtered servo input as -1 to 1 range
    float filtered_servo_angle(const struct sitl_input &input, uint8_t idx);
    float filtered_servo_range(const struct sitl_input &input, uint8_t idx);
    void filtered_servo_setup(uint8_t idx, uint16_t pwm_min, uint16_t pwm_max, float deflection_deg);

    // extrapolate sensors by a given delta time in seconds
    void extrapolate_sensors(float delta_time);

    // update external payload/sensor dynamic
    void update_external_payload(const struct sitl_input &input);

    void add_shove_forces(Vector3f &rot_accel, Vector3f &body_accel);
    void add_twist_forces(Vector3f &rot_accel);

    // get local thermal updraft
    float get_local_updraft(const Vector3d &currentPos);

    // update EAS speeds
    void update_eas_airspeed();

    // clamp support
    class Clamp {
    public:
        bool clamped(class Aircraft&, const struct sitl_input &input);  // true if the vehicle is currently clamped down
    private:
        bool currently_clamped;
        bool grab_attempted;  // avoid warning multiple times about missed grab
    } clamp;

private:
    uint64_t last_time_us;
    uint32_t frame_counter;
    uint32_t last_ground_contact_ms;
#if defined(__CYGWIN__) || defined(__CYGWIN64__)
    const uint32_t min_sleep_time{20000};
#else
    const uint32_t min_sleep_time{5000};
#endif

    struct {
        Vector3f accel_body;
        Vector3f gyro;
        Matrix3f rotation_b2e;
        Vector3d position;
        Vector3f velocity_ef;
        uint64_t last_update_us;
        Location location;
    } smoothing;

    ServoModel servo_filter[16];

    Buzzer *buzzer;
    Sprayer *sprayer;
    Gripper_Servo *gripper;
    Gripper_EPM *gripper_epm;
    Parachute *parachute;
    RichenPower *richenpower;
#if AP_SIM_LOWEHEISER_ENABLED
    Loweheiser *loweheiser;
#endif
    FETtecOneWireESC *fetteconewireesc;

    IntelligentEnergy24 *ie24;
    SIM_Precland *precland;
    class I2C *i2c;
#if AP_TEST_DRONECAN_DRIVERS
    DroneCANDevice *dronecan;
#endif
};

} // namespace SITL

#endif // AP_SIM_ENABLED
