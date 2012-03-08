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
#include "../AP_Compass/AP_Compass.h"
#include "../AP_ADC/AP_ADC.h"
#include "../AP_GPS/AP_GPS.h"
#include "../AP_IMU/AP_IMU.h"
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class AP_DCM
{
public:
	// Constructors
	AP_DCM(IMU *imu, GPS *&gps) :
		_kp_roll_pitch(0.13),
		_kp_yaw(0.8),
		_gps(gps),
		_imu(imu),
		_dcm_matrix(1, 0, 0,
			    0, 1, 0,
			    0, 0, 1),
		_health(1.),
		_toggle(0)
	{
		// base the ki values by the sensors maximum drift
		// rate. The APM2 has gyros which are much less drift
		// prone than the APM1, so we should have a lower ki,
		// which will make us less prone to increasing omegaI
		// incorrectly due to sensor noise
		_gyro_drift_limit = imu->get_gyro_drift_rate();
		_ki_roll_pitch = _gyro_drift_limit * 5;
		_ki_yaw        = _gyro_drift_limit * 8;
	}

	// Accessors

	// return the smoothed gyro vector corrected for drift
	Vector3f	get_gyro(void) {return _omega_smoothed; }
	Matrix3f	get_dcm_matrix(void) {return _dcm_matrix; }
	Matrix3f	get_dcm_transposed(void) {Matrix3f temp = _dcm_matrix;  return temp.transpose();}

	// return the current drift correction integrator value
	Vector3f	get_gyro_drift(void) {return _omega_I; }

	float		get_health(void) {return _health;}
	void		set_centripetal(bool b) {_centripetal = b;}
	bool		get_centripetal(void) {return _centripetal;}
	void		set_compass(Compass *compass);

	// Methods
	void 		update_DCM(uint8_t drift_correction_frequency=1);
	void 		update(void) { update_DCM(); }
	void 		matrix_reset(bool recover_eulers = false);

	long		roll_sensor;					// Degrees * 100
	long		pitch_sensor;					// Degrees * 100
	long		yaw_sensor;					// Degrees * 100

	float		roll;							// Radians
	float		pitch;							// Radians
	float		yaw;							// Radians

	uint8_t 	gyro_sat_count;
	uint8_t 	renorm_range_count;
	uint8_t 	renorm_blowup_count;

	// status reporting
	float		get_accel_weight(void);
	float		get_renorm_val(void);
	float		get_error_rp(void);
	float		get_error_yaw(void);

private:
	float		_kp_roll_pitch;
	float		_ki_roll_pitch;
	float		_kp_yaw;
	float		_ki_yaw;
	float		_gyro_drift_limit;          // radians/s/s
	bool		_have_initial_yaw;

	// Methods
	void 		read_adc_raw(void);
	void 		accel_adjust(Vector3f &accel);
	float 		read_adc(int select);
	void 		matrix_update(float _G_Dt);
	void 		normalize(void);
	void		check_matrix(void);
	bool	 	renorm(Vector3f const &a, Vector3f &result);
	void 		drift_correction(float deltat);
	void 		euler_angles(void);

	// members
	Compass 	* _compass;

	// note: we use ref-to-pointer here so that our caller can change the GPS without our noticing
	//       IMU under us without our noticing.
	GPS 		*&_gps;                     // note: this is a reference to a pointer owned by the caller

	IMU 		*_imu;

	Matrix3f	_dcm_matrix;

	// sum of accel vectors between drift_correction() calls
	// this allows the drift correction to run at a different rate
	// to the main DCM update code
	Vector3f 	_accel_vector;
	Vector3f 	_accel_sum;

	Vector3f 	_gyro_vector;		// Store the gyros turn rate in a vector
	Vector3f	_omega_P;		// accel Omega Proportional correction
	Vector3f	_omega_yaw_P;		// yaw Omega Proportional correction
	Vector3f 	_omega_I;		// Omega Integrator correction
	Vector3f 	_omega_integ_corr;	// Partially corrected Gyro_Vector data - used for centrepetal correction
	Vector3f 	_omega;			// Corrected Gyro_Vector data
	Vector3f 	_omega_sum;
	Vector3f 	_omega_smoothed;
	float		_health;
	bool		_centripetal;
	uint8_t		_toggle;

	// state to support status reporting
	float		_renorm_val_sum;
	uint16_t	_renorm_val_count;
	float		_error_rp_sum;
	uint16_t	_error_rp_count;
	float		_error_yaw_sum;
	uint16_t	_error_yaw_count;

	// time in micros when we last got a compass fix
	uint32_t	_compass_last_update;

	// time in millis when we last got a GPS heading
	uint32_t	_gps_last_update;

	// counter of calls to update_DCM() without drift correction
	uint8_t         _drift_correction_count;
	float		_drift_correction_time;

};

#endif



