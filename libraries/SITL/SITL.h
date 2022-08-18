#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Common/Location.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include "SIM_Buzzer.h"
#include "SIM_Gripper_EPM.h"
#include "SIM_Gripper_Servo.h"
#include "SIM_I2C.h"
#include "SIM_SPI.h"
#include "SIM_Parachute.h"
#include "SIM_Precland.h"
#include "SIM_Sprayer.h"
#include "SIM_ToneAlarm.h"
#include "SIM_EFI_MegaSquirt.h"
#include "SIM_RichenPower.h"
#include "SIM_FETtecOneWireESC.h"
#include "SIM_IntelligentEnergy24.h"
#include "SIM_Ship.h"
#include "SIM_GPS.h"
#include <AP_RangeFinder/AP_RangeFinder.h>

namespace SITL {

enum class LedLayout {
    ROWS=0,
    LUMINOUSBEE=1,
};
    
struct vector3f_array {
    uint16_t length;
    Vector3f *data;
};

struct float_array {
    uint16_t length;
    float *data;
};
    

struct sitl_fdm {
    // this is the structure passed between FDM models and the main SITL code
    uint64_t timestamp_us;
    Location home;
    double latitude, longitude; // degrees
    double altitude;  // MSL
    double heading;   // degrees
    double speedN, speedE, speedD; // m/s
    double xAccel, yAccel, zAccel;       // m/s/s in body frame
    double rollRate, pitchRate, yawRate; // degrees/s in body frame
    double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
    Quaternion quaternion;
    double airspeed; // m/s
    Vector3f velocity_air_bf; // velocity relative to airmass, body frame
    double battery_voltage; // Volts
    double battery_current; // Amps
    double battery_remaining; // Ah, if non-zero capacity
    uint8_t num_motors;
    uint32_t motor_mask;
    float rpm[32];         // RPM of all motors
    uint8_t rcin_chan_count;
    float  rcin[12];         // RC input 0..1
    double range;           // rangefinder value
    Vector3f bodyMagField;  // Truth XYZ magnetic field vector in body-frame. Includes motor interference. Units are milli-Gauss.
    Vector3f angAccel; // Angular acceleration in degrees/s/s about the XYZ body axes

    struct {
        // data from simulated laser scanner, if available
        struct vector3f_array points;
        struct float_array ranges;
    } scanner;

    float rangefinder_m[RANGEFINDER_MAX_INSTANCES];
    float airspeed_raw_pressure[2];

    struct {
        float speed;
        float direction;
    } wind_vane_apparent;

    bool is_lock_step_scheduled;

    // earthframe wind, from backends that know it
    Vector3f wind_ef;
};

// number of rc output channels
#define SITL_NUM_CHANNELS 32

class SIM {
public:

    SIM() {
        // set a default compass offset
        for (uint8_t i = 0; i < HAL_COMPASS_MAX_SENSORS; i++) {
            mag_ofs[i].set(Vector3f(5, 13, -18));
        }
        AP_Param::setup_object_defaults(this, var_info);
        AP_Param::setup_object_defaults(this, var_info2);
        AP_Param::setup_object_defaults(this, var_info3);
#if HAL_SIM_GPS_ENABLED
        AP_Param::setup_object_defaults(this, var_gps);
#endif
        AP_Param::setup_object_defaults(this, var_mag);
        AP_Param::setup_object_defaults(this, var_ins);
#ifdef SFML_JOYSTICK
        AP_Param::setup_object_defaults(this, var_sfml_joystick);
#endif // SFML_JOYSTICK
        for (uint8_t i=0; i<BARO_MAX_INSTANCES; i++) {
            AP_Param::setup_object_defaults(&baro[i], baro[i].var_info);
        }
        if (_singleton != nullptr) {
            AP_HAL::panic("Too many SITL instances");
        }
        _singleton = this;
    }

    /* Do not allow copies */
    SIM(const SIM &other) = delete;
    SIM &operator=(const SIM&) = delete;

    static SIM *_singleton;
    static SIM *get_singleton() { return _singleton; }

    enum SITL_RCFail {
        SITL_RCFail_None = 0,
        SITL_RCFail_NoPulses = 1,
        SITL_RCFail_Throttle950 = 2,
    };

    enum GPSHeading {
        GPS_HEADING_NONE = 0,
        GPS_HEADING_HDT  = 1,
        GPS_HEADING_THS  = 2,
        GPS_HEADING_KSXT = 3,
    };

    struct sitl_fdm state;

    // throttle when motors are active
    float throttle;

    // height above ground
    float height_agl;
    
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];
    static const struct AP_Param::GroupInfo var_info3[];
#if HAL_SIM_GPS_ENABLED
    static const struct AP_Param::GroupInfo var_gps[];
#endif
    static const struct AP_Param::GroupInfo var_mag[];
    static const struct AP_Param::GroupInfo var_ins[];
#ifdef SFML_JOYSTICK
    static const struct AP_Param::GroupInfo var_sfml_joystick[];
#endif //SFML_JOYSTICK

    // Board Orientation (and inverse)
    Matrix3f ahrs_rotation;
    Matrix3f ahrs_rotation_inv;

    AP_Float arspd_noise[2];  // pressure noise
    AP_Float arspd_fail[2];   // airspeed value in m/s to fail to
    AP_Float arspd_fail_pressure[2]; // pitot tube failure pressure in Pa
    AP_Float arspd_fail_pitot_pressure[2]; // pitot tube failure pressure in Pa
    AP_Float arspd_offset[2]; // airspeed sensor offset in m/s

    AP_Float mag_noise;   // in mag units (earth field is 818)
    AP_Vector3f mag_mot;  // in mag units per amp
    AP_Vector3f mag_ofs[HAL_COMPASS_MAX_SENSORS];  // in mag units
    AP_Vector3f mag_diag[HAL_COMPASS_MAX_SENSORS];  // diagonal corrections
    AP_Vector3f mag_offdiag[HAL_COMPASS_MAX_SENSORS];  // off-diagonal corrections
    AP_Int8 mag_orient[HAL_COMPASS_MAX_SENSORS];   // external compass orientation
    AP_Int8 mag_fail[HAL_COMPASS_MAX_SENSORS];   // fail magnetometer, 1 for no data, 2 for freeze
    AP_Float servo_speed; // servo speed in seconds

    AP_Float sonar_glitch;// probability between 0-1 that any given sonar sample will read as max distance
    AP_Float sonar_noise; // in metres
    AP_Float sonar_scale; // meters per volt

    AP_Float drift_speed; // degrees/second/minute
    AP_Float drift_time;  // period in minutes
    AP_Float engine_mul;  // engine multiplier
    AP_Int8  engine_fail; // engine servo to fail (0-7)

    AP_Float gps_noise[2]; // amplitude of the gps altitude error
    AP_Int16 gps_lock_time[2]; // delay in seconds before GPS gets lock
    AP_Int16 gps_alt_offset[2]; // gps alt error
    AP_Int8  gps_disable[2]; // disable simulated GPS
    AP_Int16 gps_delay_ms[2];   // delay in milliseconds
    AP_Int8  gps_type[2]; // see enum SITL::GPS::Type
    AP_Float gps_byteloss[2];// byte loss as a percent
    AP_Int8  gps_numsats[2]; // number of visible satellites
    AP_Vector3f gps_glitch[2];  // glitch offsets in lat, lon and altitude
    AP_Int8  gps_hertz[2];   // GPS update rate in Hz
    AP_Int8 gps_hdg_enabled[2]; // enable the output of a NMEA heading HDT sentence or UBLOX RELPOSNED
    AP_Float gps_drift_alt[2]; // altitude drift error
    AP_Vector3f gps_pos_offset[2];  // XYZ position of the GPS antenna phase centre relative to the body frame origin (m)
    AP_Float gps_accuracy[2];
    AP_Vector3f gps_vel_err[2]; // Velocity error offsets in NED (x = N, y = E, z = D)

    // initial offset on GPS lat/lon, used to shift origin
    AP_Float gps_init_lat_ofs;
    AP_Float gps_init_lon_ofs;
    AP_Float gps_init_alt_ofs;

    AP_Float batt_voltage; // battery voltage base
    AP_Float batt_capacity_ah; // battery capacity in Ah
    AP_Int8  rc_fail;     // fail RC input
    AP_Int8  rc_chancount; // channel count
    AP_Int8  float_exception; // enable floating point exception checks
    AP_Int8  flow_enable; // enable simulated optflow
    AP_Int16 flow_rate; // optflow data rate (Hz)
    AP_Int8  flow_delay; // optflow data delay
    AP_Int8  terrain_enable; // enable using terrain for height
    AP_Int16 pin_mask; // for GPIO emulation
    AP_Float speedup; // simulation speedup
    AP_Int8  odom_enable; // enable visual odometry data
    AP_Int8  telem_baudlimit_enable; // enable baudrate limiting on links
    AP_Float flow_noise; // optical flow measurement noise (rad/sec)
    AP_Int8  baro_count; // number of simulated baros to create
    AP_Int8  imu_count; // number of simulated IMUs to create
    AP_Int32 loop_delay; // extra delay to add to every loop
    AP_Float mag_scaling[MAX_CONNECTED_MAGS]; // scaling factor
    AP_Int32 mag_devid[MAX_CONNECTED_MAGS]; // Mag devid
    AP_Float buoyancy; // submarine buoyancy in Newtons
    AP_Int16 loop_rate_hz;

#ifdef SFML_JOYSTICK
    AP_Int8 sfml_joystick_id;
    AP_Int8 sfml_joystick_axis[8];
#endif

    // baro parameters
    class BaroParm {
    public:
        static const struct AP_Param::GroupInfo var_info[];
        AP_Float noise;  // in metres
        AP_Float drift;  // in metres per second
        AP_Float glitch; // glitch in meters
        AP_Int8  freeze; // freeze baro to last recorded altitude
        AP_Int8  disable; // disable simulated barometers
        AP_Int16 delay;  // barometer data delay in ms

        // wind coefficients
        AP_Float wcof_xp;
        AP_Float wcof_xn;
        AP_Float wcof_yp;
        AP_Float wcof_yn;
    };
    BaroParm baro[BARO_MAX_INSTANCES];

    // EFI type
    enum EFIType {
        EFI_TYPE_NONE = 0,
        EFI_TYPE_MS = 1,
    };
    
    AP_Int8  efi_type;

    // wind control
    enum WindType {
        WIND_TYPE_SQRT = 0,
        WIND_TYPE_NO_LIMIT = 1,
        WIND_TYPE_COEF = 2,
    };
    
    float wind_speed_active;
    float wind_direction_active;
    float wind_dir_z_active;
    AP_Float wind_speed;
    AP_Float wind_direction;
    AP_Float wind_turbulance;
    AP_Float wind_dir_z;
    AP_Int8  wind_type; // enum WindLimitType
    AP_Float wind_type_alt;
    AP_Float wind_type_coef;

    AP_Int16  mag_delay; // magnetometer data delay in ms
    AP_Int16  wind_delay; // windspeed data delay in ms

    // ADSB related run-time options
    AP_Int16 adsb_plane_count;
    AP_Float adsb_radius_m;
    AP_Float adsb_altitude_m;
    AP_Int8  adsb_tx;

    // Earth magnetic field anomaly
    AP_Vector3f mag_anomaly_ned; // NED anomaly vector at ground level (mGauss)
    AP_Float mag_anomaly_hgt; // height above ground where anomally strength has decayed to 1/8 of the ground level value (m)

    // Body frame sensor position offsets
    AP_Vector3f imu_pos_offset;     // XYZ position of the IMU accelerometer relative to the body frame origin (m)
    AP_Vector3f rngfnd_pos_offset;  // XYZ position of the range finder zero range datum relative to the body frame origin (m)
    AP_Vector3f optflow_pos_offset; // XYZ position of the optical flow sensor focal point relative to the body frame origin (m)
    AP_Vector3f vicon_pos_offset;   // XYZ position of the vicon sensor relative to the body frame origin (m)

    // barometer temperature control
    AP_Float temp_start;            // [deg C] Barometer start temperature
    AP_Float temp_board_offset;     // [deg C] Barometer board temperature offset from atmospheric temperature
    AP_Float temp_tconst;           // [deg C] Barometer warmup temperature time constant
    AP_Float temp_baro_factor;
    
    AP_Int8 thermal_scenario;

    // differential pressure sensor tube order
    AP_Int8 arspd_signflip;

    // weight on wheels pin
    AP_Int8 wow_pin;

    // vibration frequencies in Hz on each axis
    AP_Vector3f vibe_freq;

    // max frequency to use as baseline for adding motor noise for the gyros and accels
    AP_Float vibe_motor;

    // what harmonics to generate
    AP_Int16 vibe_motor_harmonics;

    // amplitude scaling of motor noise relative to gyro/accel noise
    AP_Float vibe_motor_scale;
    // minimum throttle for addition of ins noise
    AP_Float ins_noise_throttle_min;

    struct {
        AP_Float x;
        AP_Float y;
        AP_Float z;
        AP_Int32 t;

        uint32_t start_ms;
    } shove;

    struct {
        AP_Float x;
        AP_Float y;
        AP_Float z;
        AP_Int32 t;

        uint32_t start_ms;
    } twist;

    AP_Int8 gnd_behav;

    struct {
        AP_Int8 enable;     // 0: disabled, 1: roll and pitch, 2: roll, pitch and heave
        AP_Float length;    // m
        AP_Float amp;       // m
        AP_Float direction; // deg (direction wave is coming from)
        AP_Float speed;     // m/s
    } wave;

    struct {
        AP_Float direction; // deg (direction tide is coming from)
        AP_Float speed;     // m/s
    } tide;

    // original simulated position
    struct {
        AP_Float lat;
        AP_Float lng;
        AP_Float alt; // metres
        AP_Float hdg; // 0 to 360
    } opos;

    AP_Int8 _safety_switch_state;

    AP_HAL::Util::safety_state safety_switch_state() const {
        return (AP_HAL::Util::safety_state)_safety_switch_state.get();
    }
    void force_safety_off() {
        _safety_switch_state = (uint8_t)AP_HAL::Util::SAFETY_ARMED;
    }
    bool force_safety_on() {
        _safety_switch_state = (uint8_t)AP_HAL::Util::SAFETY_DISARMED;
        return true;
    }

    uint16_t irlock_port;

    time_t start_time_UTC;

    void simstate_send(mavlink_channel_t chan) const;
    void sim_state_send(mavlink_channel_t chan) const;

    void Log_Write_SIMSTATE();

    // convert a set of roll rates from earth frame to body frame
    static void convert_body_frame(double rollDeg, double pitchDeg,
                                   double rollRate, double pitchRate, double yawRate,
                                   double *p, double *q, double *r);

    // convert a set of roll rates from body frame to earth frame
    static Vector3f convert_earth_frame(const Matrix3f &dcm, const Vector3f &gyro);

    int i2c_ioctl(uint8_t i2c_operation, void *data) {
        return i2c_sim.ioctl(i2c_operation, data);
    }

    int spi_ioctl(uint8_t bus, uint8_t cs_pin, uint8_t spi_operation, void *data) {
        return spi_sim.ioctl(bus, cs_pin, spi_operation, data);
    }

    Sprayer sprayer_sim;

    // simulated ship takeoffs
#if AP_SIM_SHIP_ENABLED
    ShipSim shipsim;
#endif

    Gripper_Servo gripper_sim;
    Gripper_EPM gripper_epm_sim;

    Parachute parachute_sim;
    Buzzer buzzer_sim;
    I2C i2c_sim;
    SPI spi_sim;
    ToneAlarm tonealarm_sim;
    SIM_Precland precland_sim;
    RichenPower richenpower_sim;
    IntelligentEnergy24 ie24_sim;
    FETtecOneWireESC fetteconewireesc_sim;

    // ESC telemetry
    AP_Int8 esc_telem;

    struct {
        // LED state, for serial LED emulation
        struct {
            uint8_t rgb[3];
        } rgb[16][32];
        uint8_t num_leds[16];
        uint32_t send_counter;
    } led;

    AP_Int8 led_layout;

    // vicon parameters
    AP_Vector3f vicon_glitch;   // glitch in meters in vicon's local NED frame
    AP_Int8 vicon_fail;         // trigger vicon failure
    AP_Int16 vicon_yaw;         // vicon local yaw in degrees
    AP_Int16 vicon_yaw_error;   // vicon yaw error in degrees (added to reported yaw sent to vehicle)
    AP_Int8 vicon_type_mask;    // vicon message type mask (bit0:vision position estimate, bit1:vision speed estimate, bit2:vicon position estimate)
    AP_Vector3f vicon_vel_glitch;   // velocity glitch in m/s in vicon's local frame

    // get the rangefinder reading for the desired instance, returns -1 for no data
    float get_rangefinder(uint8_t instance);

    // get the apparent wind speed and direction as set by external physics backend
    float get_apparent_wind_dir() const{return state.wind_vane_apparent.direction;}
    float get_apparent_wind_spd() const{return state.wind_vane_apparent.speed;}

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    // IMU temperature calibration params
    AP_Float imu_temp_start;
    AP_Float imu_temp_end;
    AP_Float imu_temp_tconst;
    AP_Float imu_temp_fixed;
    AP_InertialSensor::TCal imu_tcal[INS_MAX_INSTANCES];
#endif

    // IMU control parameters
    AP_Float gyro_noise[INS_MAX_INSTANCES];  // in degrees/second
    AP_Vector3f gyro_scale[INS_MAX_INSTANCES];  // percentage
    AP_Float accel_noise[INS_MAX_INSTANCES]; // in m/s/s
    AP_Vector3f accel_bias[INS_MAX_INSTANCES]; // in m/s/s
    AP_Vector3f accel_scale[INS_MAX_INSTANCES]; // in m/s/s
    AP_Vector3f accel_trim;
    AP_Float accel_fail[INS_MAX_INSTANCES];  // accelerometer failure value
    // gyro and accel fail masks
    AP_Int8 gyro_fail_mask;
    AP_Int8 accel_fail_mask;

    // Sailboat sim only
    AP_Int8 sail_type;

    // Master instance to use servos from with slave instances
    AP_Int8 ride_along_master;
};

} // namespace SITL


namespace AP {
    SITL::SIM *sitl();
};

#endif // CONFIG_HAL_BOARD
