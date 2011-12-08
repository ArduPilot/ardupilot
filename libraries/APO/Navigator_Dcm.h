/*
 * Navigator_Dcm.h
 * Copyright (C) James Goppert/ Wenyao Xie 2011 james.goppert@gmail.com/ wenyaoxie@gmail.com 
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

#ifndef Navigator_Dcm_H
#define Navigator_Dcm_H

#include "AP_Navigator.h"
#include <FastSerial.h>
#include "AP_ControllerBlock.h"
#include <AP_DCM.h>

class RangeFinder;

namespace apo {

class Navigator_Dcm: public AP_Navigator {
public:
    Navigator_Dcm(AP_Board * board, const uint16_t key, const prog_char_t * name = NULL);
    virtual void calibrate();
    virtual void updateFast(float dt);
    virtual void updateSlow(float dt);
    void updateGpsLight(void);
private:
    AP_DCM _dcm;
    AP_Var_group _group;
    uint16_t _imuOffsetAddress;
    BlockLowPass _baroLowPass;
    AP_Float _groundTemperature;
    AP_Float _groundPressure;
    RangeFinder * _rangeFinderDown;
};

} // namespace apo

#endif // Navigator_Dcm_H
// vim:ts=4:sw=4:expandtab
