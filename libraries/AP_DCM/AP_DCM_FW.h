#ifndef AP_DCM_FW_h
#define AP_DCM_FW_h

#include <FastSerial.h>
#include <AP_Math.h>
#include <inttypes.h>
#include "WProgram.h"
#include <APM_Compass.h>
#include <APM_ADC.h>
#include <AP_GPS.h>
#include <AP_IMU.h>


class AP_DCM_FW
{
public:
	// Constructors
	AP_DCM_FW(GPS *GPS);										// Constructor - no magnetometer
	AP_DCM_FW(GPS *GPS, APM_Compass_Class *withCompass);		// Constructor for case with magnetometer
	
	// Accessors
	Vector3f	get_gyros(void);
	Vector3f	get_accels(void);
	Matrix3f	get_dcm_matrix(void);
	AP_IMU * get_imu(void);
	
	// Methods
	void	quick_init(uint16_t *_offset_address);
	void	gyro_init(uint16_t *_offset_address);
	void	init(uint16_t *_offset_address);
	void 	update_DCM(float _G_Dt);   

	long		roll_sensor;					// Degrees * 100
	long		pitch_sensor;					// Degrees * 100
	long		yaw_sensor;						// Degrees * 100
	float		roll;							// Radians
	float		pitch;							// Radians
	float		yaw;							// Radians
	float 		imu_health;						//Metric based on accel gain deweighting
	uint8_t 	gyro_sat_count;
	uint8_t 	adc_constraints;
	uint8_t 	renorm_sqrt_count;
	uint8_t 	renorm_blowup_count;

private:
	// Methods
	void 		read_adc_raw(void);
	void 		accel_adjust(void);
	float 		read_adc(int select);
	void 		matrix_update(float _G_Dt);
	void 		normalize(void);
	Vector3f 	renorm(Vector3f const &a, int &problem);
	void 		drift_correction(void);
	void 		euler_angles(void);

	// members
	APM_Compass_Class *_compass;
	GPS 		*_gps;
	AP_IMU		_imu;

	Matrix3f	_dcm_matrix;

	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				//Store the gyros turn rate in a vector
	Vector3f	_omega_P;					//Omega Proportional correction
	Vector3f 	_omega_I;					//Omega Integrator correction
	Vector3f 	_omega_integ_corr;			//Partially corrected Gyro_Vector data - used for centrepetal correction		
	Vector3f 	_omega;						//Corrected Gyro_Vector data
	Vector3f 	_error_roll_pitch;
	Vector3f 	_error_yaw;
	float 		_errorCourse;
	float 		_course_over_ground_x; 		//Course overground X axis
	float 		_course_over_ground_y; 		//Course overground Y axis

};

#endif


