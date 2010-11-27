#ifndef AP_IMU_h
#define AP_IMU_h

#include <FastSerial.h>
#include <AP_Math.h>
#include <inttypes.h>
#include "WProgram.h"
#include <AP_ADC.h>
#include <avr/eeprom.h>


class AP_IMU
{
public:
	// Constructors
	AP_IMU(AP_ADC * adc);									// Default Constructor
	
	// Methods
	void		quick_init(uint16_t *_offset_address);				// For air start
	void		init(uint16_t *_offset_address);					// For calibration (includes accels)
	void		gyro_init(uint16_t *_offset_address);				// Read gyro offsets, use stored accel offsets
	Vector3f	get_gyro(void);					// Radians/second
	Vector3f	get_accel(void);				// meters/seconds squared

	// Members
	uint8_t 	gyro_sat_count;
	uint8_t 	adc_constraints;
	
private:
	// Methods
	void		read_offsets(void);
	float 		gyro_temp_comp(int i, int temp) const;

	// members
	//uint16_t	_offset_address;			// EEPROM start address for saving/retrieving offsets
	float 		_adc_in[6];					// array that store the 6 ADC channels used by IMU
	float 		_adc_offset[6]; 			// Array that store the Offset of the gyros and accelerometers
	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				// Store the gyros turn rate in a vector
	AP_ADC * 	_adc; 						// Analog to digital converter pointer

	// constants
	static const uint8_t	_sensors[6];
	static const int    	_sensor_signs[9];
	static const uint8_t	_gyro_temp_ch = 3; 						// The ADC channel reading the gyro temperature
	static const float 		_gyro_temp_curve[3][3];
};

#endif
