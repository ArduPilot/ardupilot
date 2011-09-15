// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_IMU_Oilpan.h
/// @brief	IMU driver for the APM oilpan

#ifndef AP_IMU_Oilpan_h
#define AP_IMU_Oilpan_h


#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_ADC/AP_ADC.h"
#include <inttypes.h>

#include "IMU.h"

class AP_IMU_Oilpan : public IMU
{

public:
    /// Constructor
    ///
    /// Saves the ADC pointer and constructs the calibration data variable.
    ///
    /// @param  adc         Pointer to the AP_ADC instance that is connected to the gyro and accelerometer.
    /// @param  key         The AP_Var::key value we will use when loading/saving calibration data.
    ///
	AP_IMU_Oilpan(AP_ADC *adc, AP_Var::Key key) :
        _adc(adc),
	    _sensor_cal(key, PSTR("IMU_SENSOR_CAL"))
	  //  _sensor_cal(key, PSTR("IMU_SENSOR_CAL"), AP_Var::k_flag_no_auto_load)
	{}

	/// Do warm or cold start.
	///
	/// @note   For a partial-warmstart where e.g. the accelerometer calibration should be preserved
	///         but the gyro cal needs to be re-performed, start with ::init(WARM_START) to load the
	///         previous calibration settings, then force a re-calibration of the gyro with ::init_gyro.
	///
	/// @param  style   Selects the initialisation style.
	///                 COLD_START performs calibration of both the accelerometer and gyro.
	///                 WARM_START loads accelerometer and gyro calibration from a previous cold start.
	///
	virtual void		init(Start_style style = COLD_START, void (*callback)(unsigned long t) = delay);

	virtual void		save();
	virtual void		init_accel(void (*callback)(unsigned long t) = delay);
	virtual void		init_gyro(void (*callback)(unsigned long t) = delay);
	virtual bool		update(void);

	// for jason
	float		gx()				{ return _sensor_cal[0]; }
	float		gy()				{ return _sensor_cal[1]; }
	float		gz()				{ return _sensor_cal[2]; }
	float		ax()				{ return _sensor_cal[3]; }
	float		ay()				{ return _sensor_cal[4]; }
	float		az()				{ return _sensor_cal[5]; }

	void		ax(const float v)		{ _sensor_cal[3] = v; }
	void		ay(const float v)		{ _sensor_cal[4] = v; }
	void		az(const float v)		{ _sensor_cal[5] = v; }


private:
    AP_ADC              *_adc;          ///< ADC that we use for reading sensors
    AP_VarA<float,6>    _sensor_cal;    ///< Calibrated sensor offsets

    virtual void        _init_accel(void (*callback)(unsigned long t));  ///< no-save implementation
    virtual void        _init_gyro(void (*callback)(unsigned long t));   ///< no-save implementation

    float				_sensor_in(uint8_t channel, uint16_t adc_value, int temperature);
    float 		        _sensor_compensation(uint8_t channel, int temp) const;

	// constants
	static const uint8_t	_sensors[6];            ///< ADC channel mappings for the sensors
	static const int8_t    	_sensor_signs[6];       ///< ADC result sign adjustment for sensors
	static const uint8_t	_gyro_temp_ch = 3; 		///< ADC channel reading the gyro temperature
	static const float 		_gyro_temp_curve[3][3]; ///< Temperature compensation curve for the gyro

	// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
	// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
	// Tested value : 418
	//
	static const float      _gravity = 423.8;       ///< 1G in the raw data coming from the accelerometer
													// Value based on actual sample data from 20 boards

	static const float      _accel_scale = 9.80665 / 423.8; ///< would like to use _gravity here, but cannot

	// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
	// Tested values : 0.4026, ?, 0.4192
	//
	static const float      _gyro_gain_x = ToRad(0.4);     // X axis Gyro gain
	static const float      _gyro_gain_y = ToRad(0.41);    // Y axis Gyro gain
	static const float      _gyro_gain_z = ToRad(0.41);    // Z axis Gyro gain

	// Maximum possible value returned by an offset-corrected sensor channel
	//
	static const float      _adc_constraint = 900;

	// Gyro and Accelerometer calibration criterial
	//
	static const float		_gyro_total_cal_change = 4.0;		// Experimentally derived - allows for some minor motion
	static const float		_gyro_max_cal_offset = 320.0;
	static const float		_accel_total_cal_change = 4.0;
	static const float		_accel_max_cal_offset = 250.0;

};

#endif
