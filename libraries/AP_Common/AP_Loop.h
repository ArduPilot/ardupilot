/*
 * AP_Loop.h
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

#ifndef AP_Loop_H
#define AP_Loop_H

#include "AP_Vector.h"

class Loop
{
public:
    Loop() : _fptr(), _data(), _period(), _subLoops(), _timeStamp(), _load(), _dt() {};
    Loop(float frequency, void (*fptr)(void *) = NULL, void * data = NULL);
    void update();
    Vector<Loop *> & subLoops() {
        return _subLoops;
    }
    float frequency() {
        return 1.0e6/_period;
    }
    void frequency(float _frequency) {
        _period = 1e6/_frequency;
    }
    uint32_t timeStamp() {
        return _timeStamp;
    }
    float dt() {
        return _dt;
    }
    uint8_t load() {
        return _load;
    }
protected:
    void (*_fptr)(void *);
    void * _data;
    uint32_t _period;
    Vector<Loop *> _subLoops;
    uint32_t _timeStamp;
    uint8_t _load;
    float _dt;
};

#endif

// vim:ts=4:sw=4:expandtab
