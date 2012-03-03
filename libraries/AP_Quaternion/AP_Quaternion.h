#ifndef AP_Quaternion_h
#define AP_Quaternion_h

#include <AP_Math.h>
#include <inttypes.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_IMU.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class AP_Quaternion
{
public:
	// Constructor
	AP_Quaternion(IMU *imu, GPS *&gps, Compass *compass = NULL) :
		_imu(imu),
		_gps(gps),
		_compass(compass)
	{
		// initial quaternion
		SEq_1 = 1;
		SEq_2 = 0;
		SEq_3 = 0;
		SEq_4 = 0;

		// reference direction of flux in earth frame
		b_x = 0;
		b_z = -1;

		// scaled gyro drift limits
		beta = sqrt(3.0f / 4.0f) * gyroMeasError;
		zeta = sqrt(3.0f / 4.0f) * gyroMeasDrift;
	}

	// Accessors
	void		set_centripetal(bool b) {_centripetal = b;}
	bool		get_centripetal(void) {return _centripetal;}
	void		set_compass(Compass *compass);

	// Methods
	void 		update(void);

	// Euler angles (radians)
	float		roll;
	float		pitch;
	float		yaw;

	// integer Euler angles (Degrees * 100)
	int32_t		roll_sensor;
	int32_t		pitch_sensor;
	int32_t		yaw_sensor;


	// compatibility methods with DCM
	void update_DCM(void) { update(); }
	void update_DCM_fast(void) { update(); }
	Vector3f get_gyro(void) { return _gyro_smoothed; }
	Vector3f get_integrator(void) { return Vector3f(0,0,0); }
	float get_accel_weight(void) { return 0; }
	float get_renorm_val(void) { return 0; }
	float get_error_rp(void) { return 0; }
	float get_error_yaw(void) { return 0; }
	float get_health(void) { return 0; }
	void matrix_reset(void) { }
	uint8_t gyro_sat_count;
	uint8_t renorm_range_count;
	uint8_t renorm_blowup_count;

private:
	void 		update_IMU(float deltat, Vector3f &gyro, Vector3f &accel);
	void 		update_MARG(float deltat, Vector3f &gyro, Vector3f &accel, Vector3f &mag);

	bool		_have_initial_yaw;

	// Methods
	void 		accel_adjust(void);

	// members
	Compass 	* _compass;

	// note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
	//       IMU under us without our noticing.
	GPS 		*&_gps;
	IMU 		*_imu;

	// true if we are doing centripetal acceleration correction
	bool		_centripetal;

	// maximum gyroscope measurement error in rad/s (set to 5 degrees/second)
	static const float gyroMeasError = 5.0 * (M_PI/180.0);

	// maximum gyroscope drift rate in radians/s/s (set to 0.2 degrees/s/s)
	static const float gyroMeasDrift = 0.2 * (PI/180.0);

	float beta;
	float zeta;

	// quaternion elements
	float SEq_1, SEq_2, SEq_3, SEq_4;

	float b_x;
	float b_z;

	// estimate gyroscope biases error
	Vector3f gyro_bias;

	// smoothed gyro estimate
	Vector3f _gyro_smoothed;
};

#endif
