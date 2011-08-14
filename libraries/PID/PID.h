// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef PID_h
#define PID_h

#include <AP_Common.h>
#include <math.h>		// for fabs()
#define PID_FILTER_SIZE	6

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:

	/// Constructor for PID that saves its settings to EEPROM
	///
	/// @note	PIDs must be named to avoid either multiple parameters with the
	///			same name, or an overly complex constructor.
	///
	/// @param	key 			Storage key assigned to this PID.  Should be unique.
	/// @param	name			Name by which the PID is known, or NULL for an anonymous PID.
	///                         The name is prefixed to the P, I, D, IMAX variable names when
	///                         they are reported.
	/// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
	///
	PID(AP_Var::Key key,
	    const prog_char_t *name,
	    const float &initial_p = 0.0,
	    const float &initial_i = 0.0,
	    const float &initial_d = 0.0,
	    const int16_t &initial_imax = 0.0) :

		_group(key, name),
		// group, index, initial value, name
		_kp  (&_group, 0, initial_p, PSTR("P")),
		_ki  (&_group, 1, initial_i, PSTR("I")),
		_kd  (&_group, 2, initial_d, PSTR("D")),
		_imax(&_group, 3, initial_imax, PSTR("IMAX"))
	{
		// no need for explicit load, assuming that the main code uses AP_Var::load_all.
	}

	/// Constructor for PID that does not save its settings.
	///
    /// @param  name            Name by which the PID is known, or NULL for an anonymous PID.
    ///                         The name is prefixed to the P, I, D, IMAX variable names when
    ///                         they are reported.
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
	///
    PID(const prog_char_t *name,
        const float &initial_p = 0.0,
        const float &initial_i = 0.0,
        const float &initial_d = 0.0,
        const int16_t &initial_imax = 0.0) :

        _group(AP_Var::k_key_none, name),
        // group, index, initial value, name
        _kp  (&_group, 0, initial_p, PSTR("P")),
        _ki  (&_group, 1, initial_i, PSTR("I")),
        _kd  (&_group, 2, initial_d, PSTR("D")),
        _imax(&_group, 3, initial_imax, PSTR("IMAX"))
    {
	}

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

	long 	get_pi(int32_t error, uint16_t dt, float scaler = 1.0);


	/// Reset the PID integrator
	///
	void	reset_I();

	/// Load gain properties
	///
	void 	load_gains();

	/// Save gain properties
	///
	void 	save_gains();

	/// @name	parameter accessors
	//@{

	/// Overload the function call operator to permit relatively easy initialisation
	void operator() (const float p,
	                 const float i,
	                 const float d,
	                 const int16_t imaxval) {
		_kp = p; _ki = i; _kd = d; _imax = imaxval;
	}

	float	kP() const				{ return _kp.get(); }
	float	kI() const 				{ return _ki.get(); }
	float	kD() const 				{ return _kd.get(); }
	int16_t	imax() const			{ return _imax.get(); }

	void	kP(const float v)		{ _kp.set(v); }
	void	kI(const float v)		{ _ki.set(v); }
	void	kD(const float v)		{ _kd.set(v); }
	void	imax(const int16_t v)	{ _imax.set(abs(v)); }
	//void	filter_size(uint8_t v)	{ _filter_size = min(v, PID_FILTER_SIZE); }

	float	get_integrator() const	{ return _integrator; }

private:
	AP_Var_group	    _group;
	AP_Float16			_kp;
	AP_Float16			_ki;
	AP_Float16			_kd;
	AP_Int16			_imax;
	float				_filter[PID_FILTER_SIZE];
	uint8_t				_filter_index;

	float				_integrator;		///< integrator value
	int32_t				_last_error;		///< last error for derivative
	//float				_last_derivative; 	///< last derivative for low-pass filter

	/// Low pass filter cut frequency for derivative calculation.
	///
	/// 20 Hz becasue anything over that is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter.
	///
	static const uint8_t _fCut = 20;
};

#endif
