/*
 * AP_ControllerBlock.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_ControllerBlock.h"
#include <math.h>

namespace apo {

AP_ControllerBlock::AP_ControllerBlock(AP_Var_group * group, uint8_t groupStart,
                   uint8_t groupLength) :
    _group(group), _groupStart(groupStart),
    _groupEnd(groupStart + groupLength) {
}

BlockLowPass::BlockLowPass(AP_Var_group * group, uint8_t groupStart, float fCut,
             const prog_char_t * fCutLabel) :
    AP_ControllerBlock(group, groupStart, 1),
    _fCut(group, groupStart, fCut, fCutLabel ? : PSTR("fCut")),
    _y(0) {
}
float BlockLowPass::update(const float & input, const float & dt) {
    float RC = 1 / (2 * M_PI * _fCut); // low pass filter
    _y = _y + (input - _y) * (dt / (dt + RC));
    return _y;
}

BlockSaturation::BlockSaturation(AP_Var_group * group, uint8_t groupStart, float yMax,
                const prog_char_t * yMaxLabel) :
    AP_ControllerBlock(group, groupStart, 1),
    _yMax(group, groupStart, yMax, yMaxLabel ? : PSTR("yMax")) {
}
float BlockSaturation::update(const float & input) {

    // pid sum
    float y = input;

    // saturation
    if (y > _yMax)
        y = _yMax;
    if (y < -_yMax)
        y = -_yMax;
    return y;
}

BlockDerivative::BlockDerivative() :
    _lastInput(0), firstRun(true) {
}
float BlockDerivative::update(const float & input, const float & dt) {
    float derivative = (input - _lastInput) / dt;
    _lastInput = input;
    if (firstRun) {
        firstRun = false;
        return 0;
    } else
        return derivative;
}

BlockIntegral::BlockIntegral() :
    _i(0) {
}
float BlockIntegral::update(const float & input, const float & dt) {
    _i += input * dt;
    return _i;
}

BlockP::BlockP(AP_Var_group * group, uint8_t groupStart, float kP,
       const prog_char_t * kPLabel) :
    AP_ControllerBlock(group, groupStart, 1),
    _kP(group, groupStart, kP, kPLabel ? : PSTR("p")) {
}

float BlockP::update(const float & input) {
    return _kP * input;
}

BlockI::BlockI(AP_Var_group * group, uint8_t groupStart, float kI, float iMax,
       const prog_char_t * kILabel,
       const prog_char_t * iMaxLabel) :
    AP_ControllerBlock(group, groupStart, 2),
    _kI(group, groupStart, kI, kILabel ? : PSTR("i")),
    _blockSaturation(group, groupStart + 1, iMax, iMaxLabel ? : PSTR("iMax")),
    _eI(0) {
}

float BlockI::update(const float & input, const float & dt) {
    // integral
    _eI += input * dt;
    _eI = _blockSaturation.update(_eI);
    return _kI * _eI;
}

BlockD::BlockD(AP_Var_group * group, uint8_t groupStart, float kD, uint8_t dFCut,
       const prog_char_t * kDLabel,
       const prog_char_t * dFCutLabel) :
    AP_ControllerBlock(group, groupStart, 2),
    _blockLowPass(group, groupStart, dFCut,
                  dFCutLabel ? : PSTR("dFCut")),
    _kD(group, _blockLowPass.getGroupEnd(), kD,
        kDLabel ? : PSTR("d")), _eP(0) {
}
float BlockD::update(const float & input, const float & dt) {
    // derivative with low pass
    float _eD = _blockLowPass.update((input - _eP) / dt, dt);
    // proportional, note must come after derivative
    // because derivatve uses _eP as previous value
    _eP = input;
    return _kD * _eD;
}

BlockPID::BlockPID(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
         float kD, float iMax, float yMax, uint8_t dFcut) :
    AP_ControllerBlock(group, groupStart, 6),
    _blockP(group, groupStart, kP),
    _blockI(group, _blockP.getGroupEnd(), kI, iMax),
    _blockD(group, _blockI.getGroupEnd(), kD, dFcut),
    _blockSaturation(group, _blockD.getGroupEnd(), yMax) {
}

float BlockPID::update(const float & input, const float & dt) {
    return _blockSaturation.update(
               _blockP.update(input) + _blockI.update(input, dt)
               + _blockD.update(input, dt));
}

BlockPI::BlockPI(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
        float iMax, float yMax) :
    AP_ControllerBlock(group, groupStart, 4),
    _blockP(group, groupStart, kP),
    _blockI(group, _blockP.getGroupEnd(), kI, iMax),
    _blockSaturation(group, _blockI.getGroupEnd(), yMax) {
}

float BlockPI::update(const float & input, const float & dt) {

    float y = _blockP.update(input) + _blockI.update(input, dt);
    return _blockSaturation.update(y);
}

BlockPD::BlockPD(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
        float kD, float iMax, float yMax, uint8_t dFcut) :
    AP_ControllerBlock(group, groupStart, 4),
    _blockP(group, groupStart, kP),
    _blockD(group, _blockP.getGroupEnd(), kD, dFcut),
    _blockSaturation(group, _blockD.getGroupEnd(), yMax) {
}

float BlockPD::update(const float & input, const float & dt) {

    float y = _blockP.update(input) + _blockD.update(input, dt);
    return _blockSaturation.update(y);
}

BlockPIDDfb::BlockPIDDfb(AP_Var_group * group, uint8_t groupStart, float kP, float kI,
            float kD, float iMax, float yMax,
            const prog_char_t * dLabel) :
    AP_ControllerBlock(group, groupStart, 5),
    _blockP(group, groupStart, kP),
    _blockI(group, _blockP.getGroupEnd(), kI, iMax),
    _blockSaturation(group, _blockI.getGroupEnd(), yMax),
    _kD(group, _blockSaturation.getGroupEnd(), kD, dLabel ? : PSTR("d"))
{
}
float BlockPIDDfb::update(const float & input, const float & derivative,
             const float & dt) {

    float y = _blockP.update(input) + _blockI.update(input, dt) + _kD
              * derivative;
    return _blockSaturation.update(y);
}

} // namespace apo
// vim:ts=4:sw=4:expandtab
