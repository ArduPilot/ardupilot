// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef PID_h
#define PID_h

#include <stdint.h>

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:
	/// Constructor
	///
	/// A PID constructed in this fashion does not support save/restore.
	/// Gains are managed internally, and must be read/written using the
	/// accessor functions.
	///
	PID() :
		_address(0),
		_gain_array(&_local_gains[0])
	{}

	/// Constructor
	///
	/// The PID will manage gains internally, and the load/save functions
	/// will use 16 bytes of EEPROM storage to store gain values.
	///
	/// @param address	EEPROM base address at which PID parameters
	///					are stored.
	///
	PID(uint16_t address) :
		_address(address),
		_gain_array(&_local_gains[0])
	{}

	/// Constructor
	///
	/// Gain values for the PID are managed externally; load/save are a NOP.
	///
	/// @param gain_array	Address of an array of float values.  The
	///						array is used as kP, kI, kD and imax
	///						respectively.
	///
	PID(float *gain_array) :
		_gain_array(gain_array)
	{}

	/// Iterate the PID, return the new control value
	///
	/// Positive error produces positive output.
	///
	/// @param error	The measured error value
	/// @param dt		The time delta in milliseconds (note
	///					that update interval cannot be more
	///					than 65.535 seconds due to limited range
	///					of the data type).
	/// @param scaler	An arbitrary scale factor
	///
	/// @returns		The updated control output.
	///
	long 	get_pid(int32_t error, uint16_t dt, float scaler = 1.0);

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
	float	kP()			{ return _gain_array[0]; }
	float	kI()			{ return _gain_array[1]; }
	float	kD()			{ return _gain_array[2]; }
	float	imax()			{ return _gain_array[3]; }

	void	kP(const float v)		{ _gain_array[0] = v; }
	void	kI(const float v)		{ _gain_array[1] = v; }
	void	kD(const float v)		{ _gain_array[2] = v; }
	void	imax(const float v);

	// one-shot operator for setting all of the gain properties at once
	void operator ()(const float p, const float i, const float d, const float max)
	{ kP(p); kI(i); kD(d); imax(max); }
	//@}

private:
	uint16_t	_address;			///< EEPROM address for save/restore of P/I/D
	float		*_gain_array; 		///< pointer to the gains for this pid

	float		_local_gains[4];	///< local storage for gains when not globally managed

	float		_integrator;		///< integrator value
	int32_t		_last_error;		///< last error for derivative
	float		_last_derivative; 	///< last derivative for low-pass filter

	/// Low pass filter cut frequency for derivative calculation.
	///
	/// 20 Hz becasue anything over that is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter.
	///
	static const uint8_t _RC = 20; 
};

#endif
