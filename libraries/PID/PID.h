// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef PID_h
#define PID_h

#include <stdint.h>
#include <avr/eeprom.h>

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:
	/// Constructor
	///
	/// @param address	EEPROM base address at which PID parameters
	///					are stored.  Zero if the PID does not support
	///					save/restore.
	///
	PID(uint16_t address = 0) :
		_gain_array(0),
		_address(address),
		_integrator(0),
		_last_error(0),
		_last_derivative(0)
	{}

	/// Constructor
	///
	/// @param gain_array	Address of an array of floats from which
	///						gains will be loaded and to which they
	///						are saved.
	///
	PID(float *gain_array) :
		_gain_array(gain_array),
		_address(0),
		_integrator(0),
		_last_error(0),
		_last_derivative(0)
	{}

	/// Iterate the PID, return the new control value
	///
	/// Positive error produces positive output.
	///
	/// @param err		The measured error value
	/// @param dt		The time delta in milliseconds
	/// @param scaler	An arbitrary scale factor
	///
	/// @returns		The updated control output.
	///
	long 	get_pid(long err, long dt, float scaler);

	/// Reset the PID integrator
	///
	void	reset_I() {	
		_integrator = 0; 
		_last_error = 0; 
		_last_derivative = 0;
	}

	/// Load gain properties
	///
	void 	load_gains();

	/// Save gain properties
	///
	void 	save_gains();

	/// @name	parameter accessors
	//@{
	float	kP()			{ return _kp; }
	float	kI()			{ return _ki; }
	float	kD()			{ return _kd; }
	float	imax()			{ return _imax; }

	void	kP(float v)		{ _kp = v; }
	void	kI(float v)		{ _ki = v; }
	void	kD(float v)		{ _kd = v; }
	void	imax(float v);
	//@}

private:
	uint16_t	_address;		///< EEPROM address for save/restore of P/I/D
	float		*_gain_array; 	///< pointer to the gains for this pid

	float		_kp;			///< proportional gain
	float		_ki;			///< integral gain
	float		_kd;			///< derivative gain
	float		_imax;			///< integrator magnitude clamp

	float		_integrator;	///< integrator value
	int32_t		_last_error;	///< last error for derivative
	float		_last_derivative; ///< last derivative for low-pass filter

	/// low pass filter cut frequency
	/// for derivative calculation,
	/// set to 20 Hz becasue anything over that
	/// is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter
	static const uint8_t _RC = 20; 
};

#endif
