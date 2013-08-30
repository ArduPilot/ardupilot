// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
/// @file	Derivative.h
/// @brief	A class to implement a derivative (slope) filter
/// See http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/

#ifndef __DERIVATIVE_FILTER_H__
#define __DERIVATIVE_FILTER_H__

#include "FilterClass.h"
#include "FilterWithBuffer.h"

// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, uint8_t FILTER_SIZE>
class DerivativeFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    // constructor
    DerivativeFilter() : FilterWithBuffer<T,FILTER_SIZE>() {
    };

    // update - Add a new raw value to the filter, but don't recalculate
    virtual void        update(T sample, uint32_t timestamp);

    // return the derivative value
    virtual float        slope(void);

    // reset - clear the filter
    virtual void        reset();

private:
    bool            _new_data;
    float           _last_slope;

    // microsecond timestamps for samples. This is needed
    // to cope with non-uniform time spacing of the data
    uint32_t        _timestamps[FILTER_SIZE];
};

typedef DerivativeFilter<float,5> DerivativeFilterFloat_Size5;
typedef DerivativeFilter<float,7> DerivativeFilterFloat_Size7;
typedef DerivativeFilter<float,9> DerivativeFilterFloat_Size9;


#endif // __DERIVATIVE_FILTER_H__

