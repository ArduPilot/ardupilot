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
/// @file	GlitchFilter.cpp
/// @brief	A class to implement a complementry filter to smooth analog glitches.


#include "GlitchFilter.h"

GlitchFilter::GlitchFilter()
{
    
}

void GlitchFilter::init(float filter_koef) { 
    _filter_koef = filter_koef;
    reset();
}

void GlitchFilter::reset(void) { 
    _mean_value  = 0; 
    _error_count = 0;
}

bool GlitchFilter::is_glitch(const float filter_range, const float value)
{
    if (isinf(value) || isnan(value)) {
        return false;
    }

    if (isinf(filter_range) || isnan(filter_range) || (filter_range < 0)) {
        return false;
    }

    bool ret = true;

    if (is_zero(_mean_value)) {
        _mean_value = value;
    } else {
        const float d = fabsf(_mean_value - value) / (_mean_value + value);  
        float koeff = _filter_koef;
        if (d  > (filter_range * 0.000005f)) {  // check the difference from mean value outside allowed range
            ret = false;
            koeff *= (d * 10.0f);  // reduce koeff so bad sample does not change _mean_value much.
            _error_count++;
        }
        _mean_value = _mean_value * (1 - koeff) + value * koeff; // complimentary filter 1/k
    }
    return ret;
}