// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_IMU_Oilpan.h
/// @brief	IMU driver for the APM oilpan

#ifndef AP_IMU_Oilpan_h
#define AP_IMU_Oilpan_h

#include "IMU.h"

#include <AP_Math.h>
#include <AP_ADC.h>
#include <inttypes.h>

class AP_IMU_Oilpan : public IMU
{

public:
	AP_IMU_Oilpan(AP_ADC *adc, uint16_t address) :
		_adc(adc),
		_address(address)
	{}

	virtual void		init(Start_style style = COLD_START);
	virtual void		init_accel(Start_style style = COLD_START);
	virtual void		init_gyro(Start_style style = COLD_START);
	virtual bool		update(void);

	// XXX backwards compat hacks
	void 		zero_accel(void);

	void 		print_accel_offsets(void);					///< XXX debug hack
	void 		print_gyro_offsets(void);					///< XXX debug hack

	int			ax()		{ return _adc_offset[3]; }
	int			ay()		{ return _adc_offset[4]; }
	int			az()		{ return _adc_offset[5]; }

	void		ax(const int v)		{ _adc_offset[3] = v; }
	void		ay(const int v)		{ _adc_offset[4] = v; }
	void		az(const int v)		{ _adc_offset[5] = v; }

private:
	float 		_gyro_temp_comp(int i, int temp) const;
	void		_save_gyro_cal(void);
	void		_save_accel_cal(void);

	float		_gyro_in(uint8_t channel, int temperature);
	float		_accel_in(uint8_t channel);

	AP_ADC		*_adc; 					// Analog to digital converter pointer
	uint16_t	_address;				// EEPROM start address for saving/retrieving offsets
	float 		_adc_offset[6]; 		// Array that store the Offset of the gyros and accelerometers

	// XXX should not be implementing these here
	float 		read_EE_float(int address);
	void 		write_EE_float(float value, int address);

	// constants
	static const uint8_t	_sensors[6];
	static const int8_t    	_sensor_signs[6];
	static const uint8_t	_gyro_temp_ch = 3; 		// The ADC channel reading the gyro temperature
	static const float 		_gyro_temp_curve[3][3];
};

#endif
