// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	IMU.h
/// @brief	Abstract class defining the interface to a real or virtual
///         Inertial Measurement Unit.

#ifndef IMU_h
#define IMU_h

#include "../AP_Math/AP_Math.h"
#include <inttypes.h>

class IMU
{

public:
	/// Constructor
	IMU() {}

	enum Start_style {
		COLD_START = 0,
		WARM_START
	};

	/// Perform startup initialisation.
	///
	/// Called to initialise the state of the IMU.
	///
	/// For COLD_START, implementations using real sensors can assume
	/// that the airframe is stationary and nominally oriented.
	///
	/// For WARM_START, no assumptions should be made about the
	/// orientation or motion of the airframe.  Calibration should be
	/// as for the previous COLD_START call.
	///
	/// @param style	The initialisation startup style.
	///
	virtual void	init(Start_style style, void (*callback)(unsigned long t)) = 0;

	/// Perform cold startup initialisation for just the accelerometers.
	///
	/// @note This should not be called unless ::init has previously
	///       been called, as ::init may perform other work.
	///
	virtual void	init_accel(void (*callback)(unsigned long t)) = 0;

	/// Perform cold-start initialisation for just the gyros.
	///
	/// @note This should not be called unless ::init has previously
	///       been called, as ::init may perform other work
	///
	virtual void	init_gyro(void (*callback)(unsigned long t)) = 0;

	/// Give the IMU some cycles to perform/fetch an update from its
	/// sensors.
	///
	/// @returns	True if some state was updated.
	///
	virtual bool	update(void) = 0;

	/// Fetch the current gyro values
	///
	/// @returns	vector of rotational rates in radians/sec
	///
	Vector3f		get_gyro(void) { return _gyro; }

	/// Fetch the current accelerometer values
	///
	/// @returns	vector of current accelerations in m/s/s
	///
	Vector3f		get_accel(void) { return _accel; }


	/// Fetch the current accelerometer values
	///
	/// @returns	vector of current accelerations in m/s/s
	///
	Vector3f		get_accel_filtered(void) { return _accel_filtered; }

	/// return the number of seconds that the last update represents
	///
	/// @returns	number of seconds
	///
	float			get_delta_time(void) { return _ticks * (2.5/1000.0); }

	/// A count of bad sensor readings
	///
	/// @todo This should be renamed, as there's no guarantee that sensors
	///       are using ADCs, etc.
	///
	uint8_t 	adc_constraints;

protected:
	/// Most recent accelerometer reading obtained by ::update
	Vector3f		_accel;
	Vector3f		_accel_filtered;

	/// Most recent gyro reading obtained by ::update
	Vector3f		_gyro;

	/// number of 2.5ms ticks that the accel and gyro values
	/// were calculated from
	uint16_t       _ticks;
};

#endif
