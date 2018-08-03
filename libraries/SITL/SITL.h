#pragma once

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>

#include "SIM_Sprayer.h"
#include "SIM_Gripper_Servo.h"
#include "SIM_Gripper_EPM.h"
#include "SIM_Parachute.h"
#include "SIM_Precland.h"

class AP_Logger;

namespace SITL {

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
    double rollRate, pitchRate, yawRate; // degrees/s/s in body frame
    double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
    Quaternion quaternion;
    double airspeed; // m/s
    double battery_voltage; // Volts
    double battery_current; // Amps
    double rpm1;            // main prop RPM
    double rpm2;            // secondary RPM
    uint8_t rcin_chan_count;
    float  rcin[8];         // RC input 0..1
    double range;           // rangefinder value
    Vector3f bodyMagField;  // Truth XYZ magnetic field vector in body-frame. Includes motor interference. Units are milli-Gauss.
    Vector3f angAccel; // Angular acceleration in degrees/s/s about the XYZ body axes

    struct {
        // data from simulated laser scanner, if available
        struct vector3f_array points;
        struct float_array ranges;
    } scanner;
};

// number of rc output channels
#define SITL_NUM_CHANNELS 16

class SITL {
public:

    SITL() {
        // set a default compass offset
        mag_ofs.set(Vector3f(5, 13, -18));
        AP_Param::setup_object_defaults(this, var_info);
        AP_Param::setup_object_defaults(this, var_info2);
        if (_singleton != nullptr) {
            AP_HAL::panic("Too many SITL instances");
        }
        _singleton = this;
    }

    /* Do not allow copies */
    SITL(const SITL &other) = delete;
    SITL &operator=(const SITL&) = delete;

    static SITL *_singleton;
    static SITL *get_singleton() { return _singleton; }

    enum SITL_RCFail {
        SITL_RCFail_None = 0,
        SITL_RCFail_NoPulses = 1,
        SITL_RCFail_Throttle950 = 2,
    };

    enum GPSType {
        GPS_TYPE_NONE  = 0,
        GPS_TYPE_UBLOX = 1,
        GPS_TYPE_MTK   = 2,
        GPS_TYPE_MTK16 = 3,
        GPS_TYPE_MTK19 = 4,
        GPS_TYPE_NMEA  = 5,
        GPS_TYPE_SBP   = 6,
        GPS_TYPE_FILE  = 7,
        GPS_TYPE_NOVA  = 8,
        GPS_TYPE_SBP2   = 9,
    };

    struct sitl_fdm state;

    // loop update rate in Hz
    uint16_t update_rate_hz;

    // true when motors are active
    bool motors_on;

    // height above ground
    float height_agl;
    
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

    // noise levels for simulated sensors
    AP_Float baro_noise;  // in metres
    AP_Float baro_drift;  // in metres per second
    AP_Float baro_glitch; // glitch in meters
    AP_Float gyro_noise;  // in degrees/second
    AP_Vector3f gyro_scale;  // percentage
    AP_Float accel_noise; // in m/s/s
    AP_Float accel2_noise; // in m/s/s
    AP_Vector3f accel_bias; // in m/s/s
    AP_Vector3f accel2_bias; // in m/s/s
    AP_Float arspd_noise;  // in m/s
    AP_Float arspd_fail;   // 1st pitot tube failure
    AP_Float arspd2_fail;   // 2nd pitot tube failure
    AP_Float arspd_fail_pressure; // 1st pitot tube failure pressure
    AP_Float arspd_fail_pitot_pressure; // 1st pitot tube failure pressure
    AP_Float arspd2_fail_pressure; // 2nd pitot tube failure pressure
    AP_Float arspd2_fail_pitot_pressure; // 2nd pitot tube failure pressure
    AP_Float gps_noise; // amplitude of the gps altitude error
    AP_Int16 gps_lock_time; // delay in seconds before GPS gets lock
    AP_Int16 gps_alt_offset; // gps alt error
    AP_Int8  vicon_observation_history_length; // frame delay for vicon messages

    AP_Float mag_noise;   // in mag units (earth field is 818)
    AP_Float mag_error;   // in degrees
    AP_Vector3f mag_mot;  // in mag units per amp
    AP_Vector3f mag_ofs;  // in mag units
    AP_Vector3f mag_diag;  // diagonal corrections
    AP_Vector3f mag_offdiag;  // off-diagonal corrections
    AP_Int8 mag_orient;   // external compass orientation
    AP_Float servo_speed; // servo speed in seconds

    AP_Float sonar_glitch;// probablility between 0-1 that any given sonar sample will read as max distance
    AP_Float sonar_noise; // in metres
    AP_Float sonar_scale; // meters per volt

    AP_Float drift_speed; // degrees/second/minute
    AP_Float drift_time;  // period in minutes
    AP_Float engine_mul;  // engine multiplier
    AP_Int8  engine_fail; // engine servo to fail (0-7)
    AP_Int8  gps_disable; // disable simulated GPS
    AP_Int8  gps2_enable; // enable 2nd simulated GPS
    AP_Int8  gps_delay;   // delay in samples
    AP_Int8  gps_type;    // see enum GPSType
    AP_Int8  gps2_type;   // see enum GPSType
    AP_Float gps_byteloss;// byte loss as a percent
    AP_Int8  gps_numsats; // number of visible satellites
    AP_Vector3f gps_glitch;  // glitch offsets in lat, lon and altitude
    AP_Vector3f gps2_glitch; // glitch offsets in lat, lon and altitude for 2nd GPS
    AP_Int8  gps_hertz;   // GPS update rate in Hz
    AP_Float batt_voltage; // battery voltage base
    AP_Float accel_fail;  // accelerometer failure value
    AP_Int8  rc_fail;     // fail RC input
    AP_Int8  rc_chancount; // channel count
    AP_Int8  baro_disable; // disable simulated barometer
    AP_Int8  float_exception; // enable floating point exception checks
    AP_Int8  flow_enable; // enable simulated optflow
    AP_Int16 flow_rate; // optflow data rate (Hz)
    AP_Int8  flow_delay; // optflow data delay
    AP_Int8  terrain_enable; // enable using terrain for height
    AP_Int16 pin_mask; // for GPIO emulation
    AP_Float speedup; // simulation speedup
    AP_Int8  odom_enable; // enable visual odomotry data
    AP_Int8  telem_baudlimit_enable; // enable baudrate limiting on links

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
    AP_Float gps_drift_alt;
    AP_Float wind_dir_z;
    AP_Int8  wind_type; // enum WindLimitType
    AP_Float wind_type_alt;
    AP_Float wind_type_coef;

    AP_Int16  baro_delay; // barometer data delay in ms
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
    AP_Vector3f gps_pos_offset;     // XYZ position of the GPS antenna phase centre relative to the body frame origin (m)
    AP_Vector3f rngfnd_pos_offset;  // XYZ position of the range finder zero range datum relative to the body frame origin (m)
    AP_Vector3f optflow_pos_offset; // XYZ position of the optical flow sensor focal point relative to the body frame origin (m)

    // temperature control
    AP_Float temp_start;
    AP_Float temp_flight;
    AP_Float temp_tconst;
    AP_Float temp_baro_factor;
    
    // differential pressure sensor tube order
    AP_Int8 arspd_signflip;

    // weight on wheels pin
    AP_Int8 wow_pin;

    // vibration frequencies in Hz on each axis
    AP_Vector3f vibe_freq;
    
    uint16_t irlock_port;

    void simstate_send(mavlink_channel_t chan);

    void Log_Write_SIMSTATE();

    // convert a set of roll rates from earth frame to body frame
    static void convert_body_frame(double rollDeg, double pitchDeg,
                                   double rollRate, double pitchRate, double yawRate,
                                   double *p, double *q, double *r);

    // convert a set of roll rates from body frame to earth frame
    static Vector3f convert_earth_frame(const Matrix3f &dcm, const Vector3f &gyro);

    Sprayer sprayer_sim;

    Gripper_Servo gripper_sim;
    Gripper_EPM gripper_epm_sim;

    Parachute parachute_sim;
    SIM_Precland precland_sim;
};

} // namespace SITL


namespace AP {
    SITL::SITL *sitl();
};
