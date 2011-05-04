/*
 * AP_Loop.pde
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

#include "AP_Loop.h"

Loop::Loop(float _frequency, void (*fptr)(void *), void * data) :
    _fptr(fptr),
    _data(data),
    _period(1.0e6/_frequency),
    _subLoops(),
    _timeStamp(micros()),
    _load(0),
    _dt(0)
{
}

void Loop::update()
{
    // quick exit if not ready
    if (micros() - _timeStamp < _period) return;

    // update time stamp
    uint32_t timeStamp0 = _timeStamp;
    _timeStamp = micros();
    _dt = (_timeStamp - timeStamp0)/1.0e6;

    // update sub loops
    for (uint8_t i=0; i<_subLoops.getSize(); i++) _subLoops[i]->update();

    // callback function
    if (_fptr) _fptr(_data);

    // calculated load with a low pass filter
    _load = 0.9*_load + 10*(float(micros()-_timeStamp)/(_timeStamp-timeStamp0));
}

// vim:ts=4:sw=4:expandtab
