// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannel.h
/// @brief	AP_RcChannel manager

#ifndef AP_RCCHANNEL_H
#define AP_RCCHANNEL_H

#include <stdint.h>
#include "../APM_RC/APM_RC.h"
#include "../AP_Common/AP_Common.h"
#include "../AP_Common/AP_Var.h"

namespace apo {

enum rcMode_t {
    RC_MODE_IN, RC_MODE_OUT, RC_MODE_INOUT
};

/// @class	AP_RcChannel
/// @brief	Object managing one RC channel
class AP_RcChannel: public AP_Var_group {

public:

    /// Constructor
    AP_RcChannel(AP_Var::Key keyValue, const prog_char_t * name, APM_RC_Class * rc,
                 const uint8_t & ch, const uint16_t & pwmMin,
                 const uint16_t & pwmNeutral, const uint16_t & pwmMax,
                 const rcMode_t & rcMode,
                 const bool & reverse, const float & scale = 0);

    // configuration
    AP_Uint8 _ch;
    AP_Uint16 _pwmMin;
    AP_Uint16 _pwmNeutral;
    AP_Uint16 _pwmMax;
    rcMode_t _rcMode;
    AP_Bool _reverse;
    AP_Float _scale;

    // get
    uint16_t getPwm() {
        return _pwm;
    }
    uint16_t getRadioPwm();
    float getPosition() {
        return _pwmToPosition(getPwm());
    }
    float getRadioPosition() {
        return _pwmToPosition(getRadioPwm());
    }
    float getScaled() {
        return _scale*getPwm();
    }

    // set
    void setUsingRadio() {
        if (_rcMode != RC_MODE_OUT) setPwm(getRadioPwm());
    }
    void setPwm(uint16_t pwm);
    void setPosition(float position) {
        setPwm(_positionToPwm(position));
    }
    void setScaled(float val) {
        setPwm(val/_scale);
    }

protected:

    // configuration
    APM_RC_Class * _rc;

    // internal states
    uint16_t _pwm; // this is the internal state, position is just created when needed

    // private methods
    uint16_t _positionToPwm(const float & position);
    float _pwmToPosition(const uint16_t & pwm);
};

} // apo

#endif	// AP_RCCHANNEL_H
// vim:ts=4:sw=4:expandtab
