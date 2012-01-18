#ifndef AP_DCM_h
#define AP_DCM_h

#include <inttypes.h>
//#include "WProgram.h"

////////////////////////////////////////////////////////////////////////////////
// XXX HACKS
class APM_Compass {
public:
	int	Heading_X;
	int	Heading_Y;
};

typedef uint8_t byte;

class APM_ADC {
public:
	int	Ch(int c) {return ~c;};
};

extern int ground_speed;
extern int pitch;
extern int yaw;
extern int roll;
extern int roll_sensor;
extern int pitch_sensor;
extern int yaw_sensor;
#define SPEEDFILT 100

// XXX warning, many of these are nonsense just to make the compiler think
#define abs(_x)				(((_x) < 0) ? -(_x) : (_x))
#define constrain(_x, _min, _max)	(((_x) < (_min)) ? (_min) : (((_x) > (_max)) ? (_max) : (_x)))
#define sqrt(_x)			((_x) / 2) // !!!
#define radians(_x)			((_x) / (180 * 3.14))	// !!! shoot me...
#define degrees(_x)			((_x) * (180 / 3.14))
#define accel_scale(_x)			((_x) * 3) // !!!
#define gyro_scaled_X(_x)		((_x) / 3)
#define gyro_scaled_Y(_x)		((_x) / 3)
#define gyro_scaled_Z(_x)		((_x) / 3)
#define asin(_x)			((_x) * 5)
#define atan2(_x, _y)			(((_x) + (_y)) / 5)

// XXX END HACKS
////////////////////////////////////////////////////////////////////////////////

class DCM_Vector {
public:
	DCM_Vector(float v0 = 0, float v1 = 0, float v2 = 0);

	// access vector elements with obj(element)
	float&	operator()	(int x) {return _v[x];};
	float	operator()	(int x) const {return _v[x];};

	DCM_Vector operator+	(DCM_Vector const &a) const;			// add
	void operator+=		(DCM_Vector const &a);				// add
	DCM_Vector operator^	(DCM_Vector const &a) const;			// cross-product
	DCM_Vector operator*	(float scale) const;				// scale
	void operator*=		(float scale);					// scale

	float	dot_product(DCM_Vector const &v2) const;
	float	magnitude(void) const;

private:
	float	_v[3];
};

class DCM_Matrix {
public:
	DCM_Matrix(float m00 = 0, float m01 = 0, float m02 = 0,
		   float m10 = 0, float m11 = 0, float m12 = 0,
		   float m20 = 0, float m21 = 0, float m22 = 0);

	// access matrix elements with obj(x,y)
	float&	operator()	(int x, int y) {return _m[x](y);};
	float	operator()	(int x, int y) const {return _m[x](y);};

	// access matrix columns with obj(x)
	DCM_Vector& operator()	(int x) {return _m[x];};
	DCM_Vector operator()	(int x) const {return _m[x];};

	// matrix multiply
	DCM_Matrix operator*	(DCM_Matrix const &a) const;

	// matrix add
	void	operator+=	(DCM_Matrix const &a);

private:
	DCM_Vector	_m[3];
};


class AP_DCM
{
public:
	// Methods
	AP_DCM(APM_Compass *withCompass);
	void 	update_DCM(void);   //G_Dt

	// XXX these are all private (called by update_DCM only?)
	void 	read_adc_raw(void);
	void 	euler_angles(void);
	void 	matrix_update(void);
	void 	drift_correction(void);
	void 	normalize(void);
	float 	read_adc(int select);

	float 	imu_health;						//Metric based on accel gain deweighting
	byte 	gyro_sat_count;
	byte 	adc_constraints;
	byte 	renorm_sqrt_count;
	byte 	renorm_blowup_count;

private:
	// Methods
	void 	_accel_adjust(void);
	float 	_gyro_temp_comp(int i, int temp) const;
	DCM_Vector _renorm(DCM_Vector const &a, uint8_t &problem);

	// members
	APM_Compass *_compass;

	DCM_Matrix	_dcm_matrix;

	float _adc_in[6]; 		// array that store the 6 ADC channels used by IMU
	float _adc_offset[6]; 		// Array that store the Offset of the gyros and accelerometers
	float _G_Dt;			// Integration time for the gyros (DCM algorithm)
	DCM_Vector _accel_vector;	// Store the acceleration in a vector
	DCM_Vector _gyro_vector;	//Store the gyros turn rate in a vector
	DCM_Vector _omega_vector;	//Corrected Gyro_Vector data
	DCM_Vector _omega_P;		//Omega Proportional correction
	DCM_Vector _omega_I;		//Omega Integrator
	DCM_Vector _omega;
	DCM_Vector _error_roll_pitch;
	DCM_Vector _error_yaw;
	float _errorCourse;
	float _course_over_ground_x; 	//Course overground X axis
	float _course_over_ground_y; 	//Course overground Y axis

	// constants
	static const uint8_t	_sensors[6];
	static const int    	_sensor_signs[9];
	static const uint8_t	_gyro_temp_ch = 3; 						// The ADC channel reading the gyro temperature
	static const float _gyro_temp_curve[3][3];
};

#endif
