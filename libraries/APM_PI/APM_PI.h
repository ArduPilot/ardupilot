// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PI.h
/// @brief	Generic PI algorithm, with EEPROM-backed storage of constants.

#ifndef APM_PI_h
#define APM_PI_h

#include <AP_Common.h>
#include <AP_Param.h>
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
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_imax    Initial value for the imax term.4
    ///
    APM_PI(const float &    initial_p = 0.0,
           const float &    initial_i = 0.0,
           const int16_t &  initial_imax = 0.0)
    {
        _kp = initial_p;
        _ki = initial_i;
        _imax = initial_imax;
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
    //long  get_pi(int32_t error, float	 dt);
    int32_t         get_pi(int32_t error, float dt);
    int32_t         get_p(int32_t error);
    int32_t         get_i(int32_t error, float dt);


    /// Reset the PI integrator
    ///
    void        reset_I();

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const int16_t  imaxval) {
        _kp = p; _ki = i; _imax = imaxval;
    }

    float           kP() const {
        return _kp.get();
    }
    float           kI() const {
        return _ki.get();
    }
    int16_t         imax() const {
        return _imax.get();
    }

    void            kP(const float v)               {
        _kp.set(v);
    }
    void            kI(const float v)               {
        _ki.set(v);
    }
    void            imax(const int16_t v)   {
        _imax.set(abs(v));
    }
    float           get_integrator() const {
        return _integrator;
    }
    void            set_integrator(float i) {
        _integrator = i;
    }

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Int16        _imax;

    float           _integrator;                                ///< integrator value
};

#endif
