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
/// @file	GlitchFilter.h
/// @brief	A class to implement a complementry filter to smooth analog glitches.

#pragma once

#include <AP_Math/AP_Math.h>
#include "FilterClass.h"


class GlitchFilter {
public:
    GlitchFilter();

    
    CLASS_NO_COPY(GlitchFilter);


    void init(float filter_koef);
    
    void reset(void);

    bool is_glitch(const float filter_range, const float value);
    
    
protected:
    
private:
    float _filter_koef;

    float _mean_value;
    float _filter_range;
    uint32_t _error_count;
};

