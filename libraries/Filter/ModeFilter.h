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
/// @file	ModeFilter.h
/// @brief	A class to apply a mode filter which is basically picking the median value from the last x samples
///         the filter size (i.e buffer size) should always be an odd number
#pragma once

#include <inttypes.h>
#include "FilterClass.h"
#include "FilterWithBuffer.h"

template <class T, uint8_t FILTER_SIZE>
class ModeFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    ModeFilter(uint8_t return_element);

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T        apply(T sample) override;

    // get - get latest filtered value from filter (equal to the value returned by latest call to apply method)
    virtual T        get() const {
        return _output;
    }

private:
    // private methods
    uint8_t         _return_element;
    T               _output;
    void            isort(T sample, bool drop_high_sample);
    bool            drop_high_sample; // switch to determine whether to drop the highest or lowest sample when new value arrives
};

// Typedef for convenience
typedef ModeFilter<int8_t,3> ModeFilterInt8_Size3;
typedef ModeFilter<int8_t,4> ModeFilterInt8_Size4;
typedef ModeFilter<int8_t,5> ModeFilterInt8_Size5;
typedef ModeFilter<int8_t,6> ModeFilterInt8_Size6;
typedef ModeFilter<int8_t,7> ModeFilterInt8_Size7;
typedef ModeFilter<uint8_t,3> ModeFilterUInt8_Size3;
typedef ModeFilter<uint8_t,4> ModeFilterUInt8_Size4;
typedef ModeFilter<uint8_t,5> ModeFilterUInt8_Size5;
typedef ModeFilter<uint8_t,6> ModeFilterUInt8_Size6;
typedef ModeFilter<uint8_t,7> ModeFilterUInt8_Size7;
typedef ModeFilter<int16_t,3> ModeFilterInt16_Size3;
typedef ModeFilter<int16_t,4> ModeFilterInt16_Size4;
typedef ModeFilter<int16_t,5> ModeFilterInt16_Size5;
typedef ModeFilter<int16_t,6> ModeFilterInt16_Size6;
typedef ModeFilter<int16_t,7> ModeFilterInt16_Size7;
typedef ModeFilter<uint16_t,3> ModeFilterUInt16_Size3;
typedef ModeFilter<uint16_t,4> ModeFilterUInt16_Size4;
typedef ModeFilter<uint16_t,5> ModeFilterUInt16_Size5;
typedef ModeFilter<uint16_t,6> ModeFilterUInt16_Size6;
typedef ModeFilter<uint16_t,7> ModeFilterUInt16_Size7;
typedef ModeFilter<float,3> ModeFilterFloat_Size3;
typedef ModeFilter<float,4> ModeFilterFloat_Size4;
typedef ModeFilter<float,5> ModeFilterFloat_Size5;
typedef ModeFilter<float,6> ModeFilterFloat_Size6;
typedef ModeFilter<float,7> ModeFilterFloat_Size7;

// Constructor    //////////////////////////////////////////////////////////////

template <class T, uint8_t FILTER_SIZE>
ModeFilter<T,FILTER_SIZE>::ModeFilter(uint8_t return_element) :
    FilterWithBuffer<T,FILTER_SIZE>(),
    _return_element(return_element),
    drop_high_sample(true)
{
    // ensure we have a valid return_nth_element value.  if not, revert to median
    if( _return_element >= FILTER_SIZE )
        _return_element = FILTER_SIZE / 2;
};

// Public Methods //////////////////////////////////////////////////////////////

template <class T, uint8_t FILTER_SIZE>
T ModeFilter<T,FILTER_SIZE>::        apply(T sample)
{
    // insert the new items into the samples buffer
    isort(sample, drop_high_sample);

    // next time drop from the other end of the sample buffer
    drop_high_sample = !drop_high_sample;

    // return results
    if( FilterWithBuffer<T,FILTER_SIZE>::sample_index < FILTER_SIZE ) {
        // middle sample if buffer is not yet full
        return _output = FilterWithBuffer<T,FILTER_SIZE>::samples[(FilterWithBuffer<T,FILTER_SIZE>::sample_index / 2)];
    }else{
        // return element specified by user in constructor
        return _output = FilterWithBuffer<T,FILTER_SIZE>::samples[_return_element];
    }
}

//
// insertion sort - takes a new sample and pushes it into the sample array
//                  drops either the highest or lowest sample depending on the 'drop_high_sample' parameter
//
template <class T, uint8_t FILTER_SIZE>
void ModeFilter<T,FILTER_SIZE>::        isort(T new_sample, bool drop_high)
{
    int8_t        i;

    // if the buffer isn't full simply increase the #items in the buffer (i.e. sample_index)
    // the rest is the same as dropping the high sample
    if( FilterWithBuffer<T,FILTER_SIZE>::sample_index < FILTER_SIZE ) {
        FilterWithBuffer<T,FILTER_SIZE>::sample_index++;
        drop_high = true;
    }

    if( drop_high ) {      // drop highest sample from the buffer to make room for our new sample

        // start from top. Note: sample_index always points to the next open space so we start from sample_index-1
        i = FilterWithBuffer<T,FILTER_SIZE>::sample_index-1;

        // if the next element is higher than our new sample, push it up one position
        while(i > 0 && FilterWithBuffer<T,FILTER_SIZE>::samples[i-1] > new_sample) {
            FilterWithBuffer<T,FILTER_SIZE>::samples[i] = FilterWithBuffer<T,FILTER_SIZE>::samples[i-1];
            i--;
        }

        // add our new sample to the buffer
        FilterWithBuffer<T,FILTER_SIZE>::samples[i] = new_sample;

    }else{     // drop lowest sample from the buffer to make room for our new sample

        // start from the bottom
        i = 0;

        // if the element is lower than our new sample, push it down one position
        while( FilterWithBuffer<T,FILTER_SIZE>::samples[i+1] < new_sample && i < FilterWithBuffer<T,FILTER_SIZE>::sample_index-1 ) {
            FilterWithBuffer<T,FILTER_SIZE>::samples[i] = FilterWithBuffer<T,FILTER_SIZE>::samples[i+1];
            i++;
        }

        // add our new sample to the buffer
        FilterWithBuffer<T,FILTER_SIZE>::samples[i] = new_sample;
    }
}
