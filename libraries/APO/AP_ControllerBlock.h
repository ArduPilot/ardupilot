/*
 * AP_ControllerBlock.h
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

#ifndef AP_ControllerBlock_H
#define AP_ControllerBlock_H

// inclusions
#include "../AP_Common/AP_Common.h"
#include "../AP_Common/AP_Var.h"

// ArduPilotOne namespace
namespace apo {

///
// The abstract class defining a controller block.
class AP_ControllerBlock {
public:
    ///
    // Controller block constructor.
    // This creates a controller block.
    // @group The group containing the class parameters.
    // @groupStart The start of the group. Used for chaining parameters.
    // @groupEnd The end of the group.
    AP_ControllerBlock(AP_Var_group * group, uint8_t groupStart,
                       uint8_t groupLength);

    ///
    // Get the end of the AP_Var group.
    // This is used for chaining multiple AP_Var groups into one.
    uint8_t getGroupEnd() {
        return _groupEnd;
    }
protected:
    AP_Var_group * _group; /// Contains all the parameters of the class.
    uint8_t _groupStart; /// The start of the AP_Var group. Used for chaining parameters.
    uint8_t _groupEnd; /// The end of the AP_Var group.
};

///
// A low pass filter block.
// This takes a signal and smooths it out. It 
// cuts all frequencies higher than the frequency provided.
class BlockLowPass: public AP_ControllerBlock {
public:
    ///
    // The constructor.
    // @group The group containing the class parameters.
    // @groupStart The start of the group. Used for chaining parameters.
    // @fCut The cut-off frequency in Hz for smoothing.
    BlockLowPass(AP_Var_group * group, uint8_t groupStart, float fCut,
                 const prog_char_t * fCutLabel = NULL);

    ///
    // The update function.
    // @input The input signal.
    // @dt The timer interval.
    // @return The output (smoothed) signal.
    float update(const float & input, const float & dt);
protected:
    AP_Float _fCut; /// The cut-off frequency in Hz.
    float _y; /// The internal state of the low pass filter.
};

///
// This block saturates a signal.
// If the signal is above max it is set to max.
// If it is below -max it is set to -max. 
class BlockSaturation: public AP_ControllerBlock {
public:
    ///
    // Controller block constructor.
    // This creates a controller block.
    // @group The group containing the class parameters.
    // @groupStart The start of the group. Used for chaining parameters.
    // @yMax The maximum threshold.
    BlockSaturation(AP_Var_group * group, uint8_t groupStart, float yMax,
                    const prog_char_t * yMaxLabel = NULL);
    ///
    // The update function.
    // @input The input signal.
    // @return The output (saturated) signal.
    float update(const float & input);
protected:
    AP_Float _yMax; /// output saturation
};

///
// This block calculates a derivative.
class BlockDerivative {
public:
    /// The constructor.
    BlockDerivative();

    ///
    // The update function.
    // @input The input signal.
    // @return The derivative.
    float update(const float & input, const float & dt);
protected:
    float _lastInput; /// The last input to the block.
    bool firstRun; /// Keeps track of first run inorder to decide if _lastInput should be used.
};

/// This block calculates an integral.
class BlockIntegral {
public:
    /// The constructor.
    BlockIntegral();
    ///
    // The update function.
    // @input The input signal.
    // @dt The timer interval.
    // @return The output (integrated) signal.
    float update(const float & input, const float & dt);
protected:
    float _i; /// integral
};

///
// This is a proportional block with built-in gain.
class BlockP: public AP_ControllerBlock {
public:
    BlockP(AP_Var_group * group, uint8_t groupStart, float kP,
           const prog_char_t * kPLabel = NULL);
    ///
    // The update function.
    // @input The input signal.
    // @return The output signal (kP*input).
    float update(const float & input);
protected:
    AP_Float _kP; /// proportional gain
};

///
// This is a integral block with built-in gain.
class BlockI: public AP_ControllerBlock {
public:
    BlockI(AP_Var_group * group, uint8_t groupStart, float kI, float iMax,
           const prog_char_t * kILabel = NULL,
           const prog_char_t * iMaxLabel = NULL);
    float update(const float & input, const float & dt);
protected:
    AP_Float _kI;                       /// integral gain
    BlockSaturation _blockSaturation;   /// integrator saturation
    float _eI;                          /// internal integrator state
};

///
// This is a derivative block with built-in gain.
class BlockD: public AP_ControllerBlock {
public:
    BlockD(AP_Var_group * group, uint8_t groupStart, float kD, uint8_t dFCut,
           const prog_char_t * kDLabel = NULL,
           const prog_char_t * dFCutLabel = NULL);
    float update(const float & input, const float & dt);
protected:
    BlockLowPass _blockLowPass;         /// The low-pass filter block
    AP_Float _kD;                       /// The derivative gain
    float _eP;                          /// The previous state
};

///
// This is a proportional, integral, derivative block with built-in gains.
class BlockPID: public AP_ControllerBlock {
public:
    BlockPID(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
             float kD, float iMax, float yMax, uint8_t dFcut);
    float update(const float & input, const float & dt);
protected:
    BlockP _blockP;                     /// The proportional block.
    BlockI _blockI;                     /// The integral block.
    BlockD _blockD;                     /// The derivative block.
    BlockSaturation _blockSaturation;   /// The saturation block.
};

///
// This is a proportional, integral block with built-in gains.
class BlockPI: public AP_ControllerBlock {
public:
    BlockPI(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
            float iMax, float yMax);
    float update(const float & input, const float & dt);
protected:
    BlockP _blockP;                     /// The proportional block.
    BlockI _blockI;                     /// The derivative block.
    BlockSaturation _blockSaturation;   /// The saturation block.
};

///
// This is a proportional, derivative block with built-in gains.
class BlockPD: public AP_ControllerBlock {
public:
    BlockPD(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
            float kD, float iMax, float yMax, uint8_t dFcut);
    float update(const float & input, const float & dt);
protected:
    BlockP _blockP;                     /// The proportional block.
    BlockD _blockD;                     /// The derivative block.
    BlockSaturation _blockSaturation;   /// The saturation block.
};

/// PID with derivative feedback (ignores reference derivative)
class BlockPIDDfb: public AP_ControllerBlock {
public:
    BlockPIDDfb(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
                float kD, float iMax, float yMax,
                const prog_char_t * dLabel = NULL);
    float update(const float & input, const float & derivative,
                 const float & dt);
protected:
    BlockP _blockP;                     /// The proportional block.
    BlockI _blockI;                     /// The integral block.
    BlockSaturation _blockSaturation;   /// The saturation block.
    AP_Float _kD;                       /// derivative gain
};

} // apo

#endif // AP_ControllerBlock_H
// vim:ts=4:sw=4:expandtab
