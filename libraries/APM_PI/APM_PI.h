// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PI.h
/// @brief	Generic PI algorithm, with EEPROM-backed storage of constants.

#ifndef APM_PI_h
#define APM_PI_h

#include <AP_Common.h>
//#include <math.h>		// for fabs()

/// @class	APM_PI
/// @brief	Object managing one PI control
class APM_PI {
public:

	/// Constructor for PI that saves its settings to EEPROM
	///
	/// @note	PI must be named to avoid either multiple parameters with the
	///			same name, or an overly complex constructor.
	///
	/// @param	key 			Storage key assigned to this PI.  Should be unique.
	/// @param	name			Name by which the PI is known, or NULL for an anonymous PI.
	///                         The name is prefixed to the P, I, IMAX variable names when
	///                         they are reported.
	/// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_imax    Initial value for the imax term.4
	///
	APM_PI(AP_Var::Key key,
	    const prog_char_t *name,
	    const float &initial_p = 0.0,
	    const float &initial_i = 0.0,
	    const int16_t &initial_imax = 0.0) :

		_group(key, name),
		// group, index, initial value, name
		_kp  (&_group, 0, initial_p, PSTR("P")),
		_ki  (&_group, 1, initial_i, PSTR("I")),
		_imax(&_group, 3, initial_imax, PSTR("IMAX"))
	{
		// no need for explicit load, assuming that the main code uses AP_Var::load_all.
	}

	/// Constructor for PI that does not save its settings.
	///
    /// @param  name            Name by which the PI is known, or NULL for an anonymous PI.
    ///                         The name is prefixed to the P, I, IMAX variable names when
    ///                         they are reported.
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_imax    Initial value for the imax term.4
	///
    APM_PI(const prog_char_t *name,
        const float &initial_p = 0.0,
        const float &initial_i = 0.0,
        const int16_t &initial_imax = 0.0) :

        _group(AP_Var::k_key_none, name),
        // group, index, initial value, name
        _kp  (&_group, 0, initial_p, PSTR("P")),
        _ki  (&_group, 1, initial_i, PSTR("I")),
        _imax(&_group, 3, initial_imax, PSTR("IMAX"))
    {
	}

	/// Iterate the PI, return the new control value
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
	//long 	get_pi(int32_t error, float	 dt);
	int32_t get_pi(int32_t error, float dt);
	int32_t get_p(int32_t error);
	int32_t get_i(int32_t error, float dt);


	/// Reset the PI integrator
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
	                 const int16_t imaxval) {
		_kp = p; _ki = i; _imax = imaxval;
	}

	float	kP() const				{ return _kp.get(); }
	float	kI() const 				{ return _ki.get(); }
	int16_t	imax() const			{ return _imax.get(); }

	void	kP(const float v)		{ _kp.set(v); }
	void	kI(const float v)		{ _ki.set(v); }
	void	imax(const int16_t v)	{ _imax.set(abs(v)); }
	float	get_integrator() const	{ return _integrator; }
	void	set_integrator(float i)	{ _integrator = i; }

private:
	AP_Var_group	    _group;
	AP_Float16			_kp;
	AP_Float16			_ki;
	AP_Int16			_imax;

	float				_integrator;		///< integrator value
};

#endif
