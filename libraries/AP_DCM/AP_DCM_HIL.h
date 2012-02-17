#ifndef AP_DCM_HIL_H
#define AP_DCM_HIL_H

#include "../FastSerial/FastSerial.h"
#include "../AP_Math/AP_Math.h"
#include <inttypes.h>
#include "../AP_Compass/AP_Compass.h"
#include "../AP_ADC/AP_ADC.h"
#include "../AP_GPS/AP_GPS.h"
#include "../AP_IMU/AP_IMU.h"
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class AP_DCM_HIL
{
public:
	// Constructors
	AP_DCM_HIL() :
		_dcm_matrix(1, 0, 0,
					0, 1, 0,
					0, 0, 1)
	{}

	// Accessors
	Vector3f	get_gyro(void) {return _omega_integ_corr; }
	Vector3f	get_accel(void) { return _accel_vector; }
	Matrix3f	get_dcm_matrix(void) {return _dcm_matrix; }
	Matrix3f	get_dcm_transposed(void) {Matrix3f temp = _dcm_matrix;  return temp.transpose();}

	void		set_centripetal(bool b) {}
	void		set_compass(Compass *compass) {}

	// Methods
	void 		update_DCM(void) {}
	void 		update_DCM_fast(void) {}

	float		get_health(void) { return 1.0; }

	long		roll_sensor;					// Degrees * 100
	long		pitch_sensor;					// Degrees * 100
	long		yaw_sensor;						// Degrees * 100

	float		roll;							// Radians
	float		pitch;							// Radians
	float		yaw;							// Radians

	uint8_t 	gyro_sat_count;
	uint8_t 	renorm_range_count;
	uint8_t 	renorm_blowup_count;


	float	kp_roll_pitch() 		{ return 0; }
	void	kp_roll_pitch(float v) 	{  }

	float	ki_roll_pitch() 		{ return 0; }
	void	ki_roll_pitch(float v) 	{  }

	float	kp_yaw() 				{ return 0; }
	void	kp_yaw(float v) 		{  }

	float	ki_yaw() 				{ return 0; }
	void	ki_yaw(float v) 		{  }


	void 		setHil(float roll, float pitch, float yaw,
					float rollRate, float pitchRate, float yawRate);
private:
	// Methods
	Matrix3f	_dcm_matrix;
	Vector3f 	_omega_integ_corr;
	Vector3f 	_accel_vector;
};

#endif



