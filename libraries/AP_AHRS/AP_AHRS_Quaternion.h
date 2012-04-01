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

class AP_AHRS_Quaternion : public AP_AHRS
{
public:
	// Constructor
	AP_AHRS_Quaternion(IMU *imu, GPS *&gps) : AP_AHRS(imu, gps)
	{
		// scaled gyro drift limits
		beta = sqrt(3.0f / 4.0f) * gyroMeasError;
		zeta = sqrt(3.0f / 4.0f) * _gyro_drift_limit;

		// reset attitude
		reset();
	}

	// Methods
	void 		update(void);
	void 		reset(bool recover_eulers=false);

	// get corrected gyro vector
	Vector3f get_gyro(void) {
		// notice the sign reversals here
		return Vector3f(_gyro_corrected.x, _gyro_corrected.y, _gyro_corrected.z);
	}

	Vector3f get_gyro_drift(void) {
		// notice the sign reversals here. The quaternion
		// system uses a -ve gyro bias, DCM uses a +ve
		return Vector3f(-gyro_bias.x, -gyro_bias.y, -gyro_bias.z);
        }

	float get_error_rp(void);
	float get_error_yaw(void);

	// convert quaternion to a DCM matrix, used by compass
	// null offsets code
	Matrix3f get_dcm_matrix(void) {
		Matrix3f ret;
		q.rotation_matrix(ret);
		return ret;
	}

private:
	void 		update_IMU(float deltat, Vector3f &gyro, Vector3f &accel);
	void 		update_MARG(float deltat, Vector3f &gyro, Vector3f &accel, Vector3f &mag);

	bool		_have_initial_yaw;

	// Methods
	void 		accel_adjust(void);

	// maximum gyroscope measurement error in rad/s (set to 7 degrees/second)
	static const float gyroMeasError = 20.0 * (M_PI/180.0);

	// scaled tuning constants
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

	// estimate of error
	float		_error_rp_sum;
	uint16_t	_error_rp_count;
	float		_error_rp_last;
	float		_error_yaw_sum;
	uint16_t	_error_yaw_count;
	float		_error_yaw_last;
};

#endif
