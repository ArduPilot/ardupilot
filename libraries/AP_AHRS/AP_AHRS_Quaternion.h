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
	AP_Quaternion(IMU *imu, GPS *&gps) :
		_imu(imu),
		_gps(gps)
	{
		// reference direction of flux in earth frame
		b_x = 0;
		b_z = -1;

		// limit the drift to the drift rate reported by the
		// sensor driver
		gyroMeasDrift = imu->get_gyro_drift_rate();

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
	Vector3f get_gyro(void) {
		// notice the sign reversals here
		return Vector3f(-_gyro_corrected.x, -_gyro_corrected.y, _gyro_corrected.z);
	}
	Vector3f get_gyro_drift(void) {
		// notice the sign reversals here
		return Vector3f(-gyro_bias.x, -gyro_bias.y, gyro_bias.z);
        }
	float get_accel_weight(void) { return 0; }
	float get_renorm_val(void) { return 0; }
	float get_health(void) { return 0; }
	void matrix_reset(void) { }
	uint8_t gyro_sat_count;
	uint8_t renorm_range_count;
	uint8_t renorm_blowup_count;
	float get_error_rp(void);
	float get_error_yaw(void);
	Matrix3f get_dcm_matrix(void) {
		Matrix3f ret;
		q.rotation_matrix(ret);
		return ret;
	}

private:
	void 		update_IMU(float deltat, Vector3f &gyro, Vector3f &accel);
	void 		update_MARG(float deltat, Vector3f &gyro, Vector3f &accel, Vector3f &mag);
	void 		update_drift(float deltat, Vector3f &mag);

	bool		_have_initial_yaw;

	// Methods
	void 		accel_adjust(void);

	// members
	Compass 	* _compass;
	// time in microseconds of last compass update
	uint32_t        _compass_last_update;

	// note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
	//       IMU under us without our noticing.
	GPS 		*&_gps;
	IMU 		*_imu;

	// true if we are doing centripetal acceleration correction
	bool		_centripetal;

	// maximum gyroscope measurement error in rad/s (set to 7 degrees/second)
	static const float gyroMeasError = 20.0 * (M_PI/180.0);

	float gyroMeasDrift;

	float beta;
	float zeta;

	// quaternion elements
	Quaternion q;

	// magnetic flux estimates. These are used for the automatic
	// magnetometer calibration
	float b_x;
	float b_z;

	// estimate gyroscope biases error
	Vector3f gyro_bias;

	// the current corrected gyro vector
	Vector3f _gyro_corrected;

	// accel and gyro accumulators for drift correction
	Vector3f _gyro_sum;
	Vector3f _accel_sum;
	uint32_t _sum_count;

	// estimate of error
	float		_error_rp_sum;
	uint16_t	_error_rp_count;
	float		_error_yaw_sum;
	uint16_t	_error_yaw_count;
};

#endif
