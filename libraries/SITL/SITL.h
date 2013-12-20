/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __SITL_H__
#define __SITL_H__

#include <AP_Param.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>

struct PACKED sitl_fdm {
	// this is the packet sent by the simulator
	// to the APM executable to update the simulator state
	// All values are little-endian
	double latitude, longitude; // degrees
	double altitude;  // MSL
	double heading;   // degrees
	double speedN, speedE, speedD; // m/s
	double xAccel, yAccel, zAccel;       // m/s/s in body frame
	double rollRate, pitchRate, yawRate; // degrees/s/s in earth frame
	double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
	double airspeed; // m/s
	uint32_t magic; // 0x4c56414f
};


class SITL
{
public:
    
    SITL() {
        AP_Param::setup_object_defaults(this, var_info);        
    }

    enum GPSType { 
        GPS_TYPE_NONE  = 0,
        GPS_TYPE_UBLOX = 1,
        GPS_TYPE_MTK   = 2,
        GPS_TYPE_MTK16 = 3,
        GPS_TYPE_MTK19 = 4,
        GPS_TYPE_NMEA  = 5
    };

	struct sitl_fdm state;

	static const struct AP_Param::GroupInfo var_info[];

	// noise levels for simulated sensors
	AP_Float baro_noise;  // in metres
	AP_Float baro_drift;  // in metres per second
	AP_Float gyro_noise;  // in degrees/second
	AP_Float accel_noise; // in m/s/s
	AP_Float aspd_noise;  // in m/s
	AP_Float mag_noise;   // in mag units (earth field is 818)
	AP_Float mag_error;   // in degrees
    AP_Float servo_rate;  // servo speed in degrees/second

    AP_Float sonar_glitch;// probablility between 0-1 that any given sonar sample will read as max distance
    AP_Float sonar_noise; // in metres

	AP_Float drift_speed; // degrees/second/minute
	AP_Float drift_time;  // period in minutes
    AP_Float engine_mul;  // engine multiplier
	AP_Int8  gps_disable; // disable simulated GPS
	AP_Int8  gps_delay;   // delay in samples
    AP_Int8  gps_type;    // see enum GPSType
    AP_Float gps_byteloss;// byte loss as a percent
    AP_Int8  gps_numsats; // number of visible satellites
    AP_Vector3f  gps_glitch;  // glitch offsets in lat, lon and altitude
    AP_Int8  gps_hertz;   // GPS update rate in Hz
    AP_Float batt_voltage; // battery voltage base
    AP_Float accel_fail;  // accelerometer failure value
	AP_Int8  rc_fail;     // fail RC input

    // wind control
    AP_Float wind_speed;
    AP_Float wind_direction;
    AP_Float wind_turbulance;
    
	void simstate_send(mavlink_channel_t chan);

	// convert a set of roll rates from earth frame to body frame
	static void convert_body_frame(double rollDeg, double pitchDeg,
				       double rollRate, double pitchRate, double yawRate,
				       double *p, double *q, double *r);
};

#endif // __SITL_H__
