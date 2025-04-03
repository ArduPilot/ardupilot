#pragma once


#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>


class AC_ADRC {
public:
    AC_ADRC(float B00, float dt);

    CLASS_NO_COPY(AC_ADRC);

    float update_all(float target, float measurement, bool limit);
    void reset_eso(float measurement);
    float fal(float e, float alpha, float delta);
    float sign(float x);
    // Reset filter
    void reset_filter() {
        _flags.reset_filter = true;
    }

    // flags
    struct ap_adrc_flags {
        bool reset_filter :1; // true when input filter should be reset during next call to set_input
    } _flags;

    static const struct AP_Param::GroupInfo var_info[];
protected:

    // parameters
    AP_Float _wc;          // Response bandwidth in rad/s
    AP_Float _wo;          // State estimation bandwidth in rad/s
    AP_Float _b0;          // Control gain
    AP_Float _limit;
    AP_Float _delta;
    AP_Int8  _order;

    // ESO interal variables
    float _z1;
    float _z2;
    float _z3;


    float _dt;

};
