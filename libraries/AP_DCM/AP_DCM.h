#ifndef AP_DCM_h
#define AP_DCM_h

// temporarily include all other classes here
// since this naming is a bit off from the
// convention and the AP_DCM should be the top
// header file
#include "AP_DCM_HIL.h"

#include "../FastSerial/FastSerial.h"
#include "../AP_Math/AP_Math.h"
#include <inttypes.h>
#include "WProgram.h"
#include "../AP_Compass/AP_Compass.h"
#include "../AP_ADC/AP_ADC.h"
#include "../AP_GPS/AP_GPS.h"
#include "../AP_IMU/AP_IMU.h"


class AP_DCM
{
public:
	// Constructors
	AP_DCM(IMU *imu, GPS *&gps, Compass *withCompass = NULL) :
		_compass(withCompass),
		_gps(gps),
		_imu(imu),
		_dcm_matrix(1, 0, 0,
					0, 1, 0,
					0, 0, 1),
		_course_over_ground_x(0),
		_course_over_ground_y(1),
		_health(1.),
		_kp_roll_pitch(0.05967),
		_ki_roll_pitch(0.00001278),
		_kp_yaw(0.8), // .8
		_ki_yaw(0.00004), // 0.00004
		_toggle(0),
		_clamp(3)
	{}

	// Accessors
	Vector3f	get_gyro(void) {return _omega_integ_corr; }		// We return the raw gyro vector corrected for bias
	Vector3f	get_accel(void) { return _accel_vector; }
	Matrix3f	get_dcm_matrix(void) {return _dcm_matrix; }
	Matrix3f	get_dcm_transposed(void) {Matrix3f temp = _dcm_matrix;  return temp.transpose();}
	Vector3f	get_integrator(void) {return _omega_I; }		// We return the current drift correction integrator values

	float		get_health(void) {return _health;}
	void		set_centripetal(bool b) {_centripetal = b;}
	bool		get_centripetal(void) {return _centripetal;}
	void		set_compass(Compass *compass);

	// Methods
	void 		update_DCM(void);
	void 		update_DCM_fast(void);
	void 		matrix_reset(void);

	long		roll_sensor;					// Degrees * 100
	long		pitch_sensor;					// Degrees * 100
	long		yaw_sensor;						// Degrees * 100

	float		roll;							// Radians
	float		pitch;							// Radians
	float		yaw;							// Radians

	uint8_t 	gyro_sat_count;
	uint8_t 	renorm_sqrt_count;
	uint8_t 	renorm_blowup_count;

	float	kp_roll_pitch() 		{ return _kp_roll_pitch; }
	void	kp_roll_pitch(float v) 	{ _kp_roll_pitch = v; }

	float	ki_roll_pitch() 		{ return _ki_roll_pitch; }
	void	ki_roll_pitch(float v) 	{ _ki_roll_pitch = v; }

	float	kp_yaw() 				{ return _kp_yaw; }
	void	kp_yaw(float v) 		{ _kp_yaw = v; }

	float	ki_yaw() 				{ return _ki_yaw; }
	void	ki_yaw(float v) 		{ _ki_yaw = v; }

	static const float kDCM_kp_rp_high 		= 0.15;
	static const float kDCM_kp_rp_medium	= 0.05967;
	static const float kDCM_kp_rp_low		= 0.01;
	int8_t		_clamp;


private:
	float		_kp_roll_pitch;
	float		_ki_roll_pitch;
	float		_kp_yaw;
	float		_ki_yaw;

	// Methods
	void 		read_adc_raw(void);
	void 		accel_adjust(void);
	float 		read_adc(int select);
	void 		matrix_update(float _G_Dt);
	void 		normalize(void);
	Vector3f 	renorm(Vector3f const &a, int &problem);
	void 		drift_correction(void);
	void 		euler_angles(void);

	void 		euler_rp(void);
	void 		euler_yaw(void);


	// members
	Compass 	* _compass;

	// note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
	//       IMU under us without our noticing.
	GPS 		*&_gps;                     // note: this is a reference to a pointer owned by the caller

	IMU 		*_imu;

	Matrix3f	_dcm_matrix;

	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				// Store the gyros turn rate in a vector
	Vector3f	_omega_P;					// Omega Proportional correction
	Vector3f 	_omega_I;					// Omega Integrator correction
	Vector3f 	_omega_integ_corr;			// Partially corrected Gyro_Vector data - used for centrepetal correction
	Vector3f 	_omega;						// Corrected Gyro_Vector data
	Vector3f 	_error_roll_pitch;
	Vector3f 	_error_yaw;
	float 		_errorCourse;
	float 		_course_over_ground_x; 		// Course overground X axis
	float 		_course_over_ground_y; 		// Course overground Y axis
	float		_health;
	bool		_centripetal;
	uint8_t		_toggle;
};

#endif



