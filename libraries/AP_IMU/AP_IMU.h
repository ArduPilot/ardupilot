#ifndef AP_IMU_h
#define AP_IMU_h

//#include <FastSerial.h>
#include <AP_Math.h>
#include <inttypes.h>
#include "WProgram.h"
#include <APM_ADC.h>


class AP_IMU
{
public:
	// Constructors
	AP_IMU();									// Default Constructor
	
	// Methods
	void		quick_init(void);				// For air start
	void		init(void);						// For ground start
	Vector3f	get_gyro(void);					// Radians/second
	Vector3f	get_accel(void);				// meters/seconds squared

	// Members
	uint8_t 	gyro_sat_count;
	uint8_t 	adc_constraints;
	
private:
	// Methods
	void 		read_adc_raw(void);
	float 		_gyro_temp_comp(int i, int temp) const;
	float 		read_adc(int select);

	// members
	float 		_adc_in[6];					// array that store the 6 ADC channels used by IMU
	float 		_adc_offset[6]; 			// Array that store the Offset of the gyros and accelerometers
	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				// Store the gyros turn rate in a vector

	// constants
	static const uint8_t	_sensors[6];
	static const int    	_sensor_signs[9];
	static const uint8_t	_gyro_temp_ch = 3; 						// The ADC channel reading the gyro temperature
	static const float 		_gyro_temp_curve[3][3];
};

#endif


