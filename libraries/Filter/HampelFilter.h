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
/// @file	HampelFilter.h
/// @brief	A Hampel test based filter with flexible output calculation (median or subset average), defaults to median if no argument provided.
#pragma once

#include <inttypes.h>
#include "FilterClass.h"
#include "FilterWithBuffer.h"

template <class T, uint8_t FILTER_SIZE>
class HampelFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    HampelFilter(uint8_t first_output_element, uint8_t last_output_element);
    HampelFilter(uint8_t output_element);
    HampelFilter();

    virtual T apply(T sample) override;

    // pass_sample - fast sample insertion method, skips buffer initialization (equivalent to initializing with 0's)
    //               and does not compute output, output will be computed on next get or apply
    void pass_sample(T sample);

    virtual T get()
    {
        if (output_ready) {
            return output;
        } else {
            return calculate();
        }
    }
    virtual void reset() override
    {
        initialized = false;
    }

private:
    uint8_t first_output_element;
    uint8_t last_output_element;
    T       output;
    bool    initialized;
    bool    output_ready;

    // get_median  - takes sorted array of FILTER_SIZE elements and returns median for odd or even FILTER_SIZE
    T get_median(T *buf);
    T calculate();
};
