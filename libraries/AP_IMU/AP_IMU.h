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
	AP_IMU(AP_ADC *adc, uint16_t address) :
		_adc(adc), 
		_address(address)
	{}
	
	// Methods
	void		init(void);					// inits both
	void		init_accel(void);			// just Accels
	void		init_gyro(void);			// just gyros
	void 		zero_accel(void);
	
	void		load_gyro_eeprom(void);
	void		save_gyro_eeprom(void);
	void		load_accel_eeprom(void);
	void		save_accel_eeprom(void);
	void 		print_accel_offsets(void);
	void 		print_gyro_offsets(void);
	
	void		ax(const int v)		{ _adc_offset[3] = v; }
	void		ay(const int v)		{ _adc_offset[4] = v; }
	void		az(const int v)		{ _adc_offset[5] = v; }


	// raw ADC values - called by DCM
	Vector3f	get_gyro(void);									// Radians/second
	Vector3f	get_accel(void);								// meters/seconds squared
				
	// Members
	uint8_t 	adc_constraints;		// a check of how many times we get non-sensical values
	
private:
	// Methods
	void		read_offsets(void);
	float 		gyro_temp_comp(int i, int temp) const;

	// members
	uint16_t	_address;					// EEPROM start address for saving/retrieving offsets
	float 		_adc_in[6];					// array that store the 6 ADC channels used by IMU
	float 		_adc_offset[6]; 			// Array that store the Offset of the gyros and accelerometers
	Vector3f 	_accel_vector;				// Store the acceleration in a vector
	Vector3f 	_gyro_vector;				// Store the gyros turn rate in a vector
	AP_ADC		* _adc; 					// Analog to digital converter pointer
	
	float 		read_EE_float(int address);
	void 		write_EE_float(float value, int address);

	// constants
	static const uint8_t	_sensors[6];
	static const int    	_sensor_signs[9];
	static const uint8_t	_gyro_temp_ch = 3; 						// The ADC channel reading the gyro temperature
	static const float 		_gyro_temp_curve[3][3];
};

#endif
