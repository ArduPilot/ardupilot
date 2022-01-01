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

void GlitchFilter::init(const float filter_koef, const float scaling, float penalty) { 
    
    _filter_koef = filter_koef;
    _scaling = scaling;

    if (penalty < 1.0f) {
        penalty = 1.0f;
    }
    
    _penalty = penalty;

    reset();
}

void GlitchFilter::reset(void) { 
    _mean_value  = 0; 
    _error_count = 0;
}


uint32_t GlitchFilter::get_error_count() {
    return _error_count;
}

bool GlitchFilter::is_glitch(const float filter_range, const float value)
{
    if (isinf(value) || isnan(value)) {
        return true;
    }

    
    bool ret = false;

    if (is_zero(_mean_value)) {
        _mean_value = value;
    } else {
        // percentage of difference from mean = delta change / mean;
        const float diff_without_scaling = 200 * fabsf(_mean_value - value) / (_mean_value + value); // diff divide by mean value in percent 
        const float diff = _scaling * diff_without_scaling;  
        float koeff = _filter_koef;
        
              
        if (diff  > (filter_range)) {  // check the difference from mean value outside allowed range
            ret = true;
            koeff /= (diff * _penalty);  // reduce koeff so bad sample does not change _mean_value much.
            _error_count++;
        }
        _mean_value = _mean_value * (1 - koeff) + value * koeff; // complimentary filter 1/k
    }
    return ret;
}