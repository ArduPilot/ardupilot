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


    void init(const float filter_koef, const float scaling, float penalty);
    
    void reset(void);

    // filter_range >=0 : it is equal to percentage of absolute change between diff (mean_old - value) / mean divided by 2
    // it is divided by 2 to reduce unnecessary operation and replace it with a constant.
    // if it is a glitch then returns true.
    bool is_glitch(const float filter_range, const float value);
    uint32_t get_error_count();
    
protected:
    
private:
    // ratio used by complementry filter between >0 & <1
    float _filter_koef;
    // scale the percentage to give reasonable meaning.
    //e.g. for pressure. if you want difference between values 99593.4 & 99594.4 to be translated as 1% 
    // then scalling = 10000.0f
    float _scaling;
    // reduce _filter_koef when difference exceedes threshold so it does not affect the mean.
    float _penalty;
    // depends heavily on what on data being processed. however it is always >0 and in <100 unless you accept a raise +100% then you can ive a higher values.
    // value of zero disables the filter.
    float _filter_range;
    float _mean_value;
    uint32_t _error_count;
};

