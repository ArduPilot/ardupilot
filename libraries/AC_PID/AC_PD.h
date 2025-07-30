#pragma once

/*
 Generic PD for tailsitter angle control.
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AC_PD {
public:
// Structure to hold default values for PD parameters from the AttitudeControl class
    struct Defaults {
        float p;
        float d;
        float alpha;
    };

    // Constructors
    AC_PD(float initial_p, float initial_i, float alpha);
    AC_PD(const AC_PD::Defaults &defaults) :
        AC_PD(
            defaults.p,
            defaults.d,
            defaults.alpha
            )
        { }
    CLASS_NO_COPY(AC_PD);

    // update controller
    float update(float err, float dt);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];
    /// Save gain properties
    ///
    void        save_gains();

    float get_P() const {
        return output_P;
    }
    float get_D() const {
        return derivative*kD;
    }
    float get_Derivative() const {
        return derivative;
    }
    float get_kP() const {
        return kP;
    }
    float get_kD() const {
        return kD;
    }
    AP_Float& get_kP_ref() { return kP; }
    void set_kP(const float v) { kP.set(v); }

protected:
    AP_Float        kP;
    AP_Float        kD;
    AP_Float        alpha;
    float           derivative;
    float           output_P;

private:
    const float default_kp;
    const float default_kd;
    const float default_alpha;

};
