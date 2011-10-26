/*
 * AP_Controller.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Controller_H
#define AP_Controller_H

#include <inttypes.h>
#include "../GCS_MAVLink/GCS_MAVLink.h"
#include <math.h>
#include "../AP_Common/AP_Var.h"
#include "AP_Var_keys.h"

class AP_Var_group;

namespace apo {

class AP_HardwareAbstractionLayer;
class AP_Guide;
class AP_Navigator;
class Menu;
class AP_ArmingMechanism;

/// Controller class
class AP_Controller {
public:
    AP_Controller(AP_Navigator * nav, AP_Guide * guide,
                  AP_HardwareAbstractionLayer * hal,
                  AP_ArmingMechanism * armingMechanism,
                  const uint8_t _chMode,
                  const uint16_t key = k_cntrl,
                  const prog_char_t * name = NULL);
    virtual void update(const float dt);
    void setAllRadioChannelsToNeutral();
    void setAllRadioChannelsManually();
    virtual void setMotors();
    virtual void setMotorsActive() = 0;
    virtual void setMotorsEmergency() {
        setAllRadioChannelsToNeutral();
    };
    virtual void setMotorsStandby() {
        setAllRadioChannelsToNeutral();
    };
    virtual void manualLoop(const float dt) {
        setAllRadioChannelsToNeutral();
    };
    virtual void autoLoop(const float dt) {
        setAllRadioChannelsToNeutral();
    };
    AP_Uint8 getMode() {
        return _mode;
    }
protected:
    AP_Navigator * _nav;
    AP_Guide * _guide;
    AP_HardwareAbstractionLayer * _hal;
    AP_ArmingMechanism * _armingMechanism;
    uint8_t _chMode;
    AP_Var_group _group;
    AP_Uint8 _mode;
};

class AP_ControllerBlock {
public:
    AP_ControllerBlock(AP_Var_group * group, uint8_t groupStart,
                       uint8_t groupLength) :
        _group(group), _groupStart(groupStart),
        _groupEnd(groupStart + groupLength) {
    }
    uint8_t getGroupEnd() {
        return _groupEnd;
    }
protected:
    AP_Var_group * _group; /// helps with parameter management
    uint8_t _groupStart;
    uint8_t _groupEnd;
};

class BlockLowPass: public AP_ControllerBlock {
public:
    BlockLowPass(AP_Var_group * group, uint8_t groupStart, float fCut,
                 const prog_char_t * fCutLabel = NULL) :
        AP_ControllerBlock(group, groupStart, 1),
        _fCut(group, groupStart, fCut, fCutLabel ? : PSTR("fCut")),
        _y(0) {
    }
    float update(const float & input, const float & dt) {
        float RC = 1 / (2 * M_PI * _fCut); // low pass filter
        _y = _y + (input - _y) * (dt / (dt + RC));
        return _y;
    }
protected:
    AP_Float _fCut;
    float _y;
};

class BlockSaturation: public AP_ControllerBlock {
public:
    BlockSaturation(AP_Var_group * group, uint8_t groupStart, float yMax,
                    const prog_char_t * yMaxLabel = NULL) :
        AP_ControllerBlock(group, groupStart, 1),
        _yMax(group, groupStart, yMax, yMaxLabel ? : PSTR("yMax")) {
    }
    float update(const float & input) {

        // pid sum
        float y = input;

        // saturation
        if (y > _yMax)
            y = _yMax;
        if (y < -_yMax)
            y = -_yMax;
        return y;
    }
protected:
    AP_Float _yMax; /// output saturation
};

class BlockDerivative {
public:
    BlockDerivative() :
        _lastInput(0), firstRun(true) {
    }
    float update(const float & input, const float & dt) {
        float derivative = (input - _lastInput) / dt;
        _lastInput = input;
        if (firstRun) {
            firstRun = false;
            return 0;
        } else
            return derivative;
    }
protected:
    float _lastInput; /// last input
    bool firstRun;
};

class BlockIntegral {
public:
    BlockIntegral() :
        _i(0) {
    }
    float update(const float & input, const float & dt) {
        _i += input * dt;
        return _i;
    }
protected:
    float _i; /// integral
};

class BlockP: public AP_ControllerBlock {
public:
    BlockP(AP_Var_group * group, uint8_t groupStart, float kP,
           const prog_char_t * kPLabel = NULL) :
        AP_ControllerBlock(group, groupStart, 1),
        _kP(group, groupStart, kP, kPLabel ? : PSTR("p")) {
    }

    float update(const float & input) {
        return _kP * input;
    }
protected:
    AP_Float _kP; /// proportional gain
};

class BlockI: public AP_ControllerBlock {
public:
    BlockI(AP_Var_group * group, uint8_t groupStart, float kI, float iMax,
           const prog_char_t * kILabel = NULL,
           const prog_char_t * iMaxLabel = NULL) :
        AP_ControllerBlock(group, groupStart, 2),
        _kI(group, groupStart, kI, kILabel ? : PSTR("i")),
        _blockSaturation(group, groupStart + 1, iMax, iMaxLabel ? : PSTR("iMax")),
        _eI(0) {
    }

    float update(const float & input, const float & dt) {
        // integral
        _eI += input * dt;
        _eI = _blockSaturation.update(_eI);
        return _kI * _eI;
    }
protected:
    AP_Float _kI; /// integral gain
    BlockSaturation _blockSaturation; /// integrator saturation
    float _eI; /// integral of input
};

class BlockD: public AP_ControllerBlock {
public:
    BlockD(AP_Var_group * group, uint8_t groupStart, float kD, uint8_t dFCut,
           const prog_char_t * kDLabel = NULL,
           const prog_char_t * dFCutLabel = NULL) :
        AP_ControllerBlock(group, groupStart, 2),
        _blockLowPass(group, groupStart, dFCut,
                      dFCutLabel ? : PSTR("dFCut")),
        _kD(group, _blockLowPass.getGroupEnd(), kD,
            kDLabel ? : PSTR("d")), _eP(0) {
    }
    float update(const float & input, const float & dt) {
        // derivative with low pass
        float _eD = _blockLowPass.update((_eP - input) / dt, dt);
        // proportional, note must come after derivative
        // because derivatve uses _eP as previous value
        _eP = input;
        return _kD * _eD;
    }
protected:
    BlockLowPass _blockLowPass;
    AP_Float _kD; /// derivative gain
    float _eP; /// input
};

class BlockPID: public AP_ControllerBlock {
public:
    BlockPID(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
             float kD, float iMax, float yMax, uint8_t dFcut) :
        AP_ControllerBlock(group, groupStart, 6),
        _blockP(group, groupStart, kP),
        _blockI(group, _blockP.getGroupEnd(), kI, iMax),
        _blockD(group, _blockI.getGroupEnd(), kD, dFcut),
        _blockSaturation(group, _blockD.getGroupEnd(), yMax) {
    }

    float update(const float & input, const float & dt) {
        return _blockSaturation.update(
                   _blockP.update(input) + _blockI.update(input, dt)
                   + _blockD.update(input, dt));
    }
protected:
    BlockP _blockP;
    BlockI _blockI;
    BlockD _blockD;
    BlockSaturation _blockSaturation;
};

class BlockPI: public AP_ControllerBlock {
public:
    BlockPI(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
            float iMax, float yMax) :
        AP_ControllerBlock(group, groupStart, 4),
        _blockP(group, groupStart, kP),
        _blockI(group, _blockP.getGroupEnd(), kI, iMax),
        _blockSaturation(group, _blockI.getGroupEnd(), yMax) {
    }

    float update(const float & input, const float & dt) {

        float y = _blockP.update(input) + _blockI.update(input, dt);
        return _blockSaturation.update(y);
    }
protected:
    BlockP _blockP;
    BlockI _blockI;
    BlockSaturation _blockSaturation;
};

class BlockPD: public AP_ControllerBlock {
public:
    BlockPD(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
            float kD, float iMax, float yMax, uint8_t dFcut) :
        AP_ControllerBlock(group, groupStart, 4),
        _blockP(group, groupStart, kP),
        _blockD(group, _blockP.getGroupEnd(), kD, dFcut),
        _blockSaturation(group, _blockD.getGroupEnd(), yMax) {
    }

    float update(const float & input, const float & dt) {

        float y = _blockP.update(input) + _blockD.update(input, dt);
        return _blockSaturation.update(y);
    }
protected:
    BlockP _blockP;
    BlockD _blockD;
    BlockSaturation _blockSaturation;
};

/// PID with derivative feedback (ignores reference derivative)
class BlockPIDDfb: public AP_ControllerBlock {
public:
    BlockPIDDfb(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
                float kD, float iMax, float yMax, float dFCut,
                const prog_char_t * dFCutLabel = NULL,
                const prog_char_t * dLabel = NULL) :
        AP_ControllerBlock(group, groupStart, 5),
        _blockP(group, groupStart, kP),
        _blockI(group, _blockP.getGroupEnd(), kI, iMax),
        _blockSaturation(group, _blockI.getGroupEnd(), yMax),
        _blockLowPass(group, _blockSaturation.getGroupEnd(), dFCut,
                      dFCutLabel ? : PSTR("dFCut")),
        _kD(group, _blockLowPass.getGroupEnd(), kD, dLabel ? : PSTR("d"))
    {
    }
    float update(const float & input, const float & derivative,
                 const float & dt) {

        float y = _blockP.update(input) + _blockI.update(input, dt) - _kD
                  * _blockLowPass.update(derivative,dt);
        return _blockSaturation.update(y);
    }
protected:
    BlockP _blockP;
    BlockI _blockI;
    BlockSaturation _blockSaturation;
    BlockLowPass _blockLowPass;
    AP_Float _kD; /// derivative gain
};

} // apo

#endif // AP_Controller_H
// vim:ts=4:sw=4:expandtab
