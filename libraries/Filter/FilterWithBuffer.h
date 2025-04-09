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
/// @file	FilterWithBuffer.h
/// @brief	A filter with a buffer.
///         This is implemented separately to the base Filter class to get around
///         restrictions caused by the use of templates which makes different sizes essentially
///         completely different classes
#pragma once

#include "FilterClass.h"

template <class T, uint8_t FILTER_SIZE>
class FilterWithBuffer : public Filter<T>
{
public:
    // constructor
    FilterWithBuffer();

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample) override;

    // reset - clear the filter
    virtual void reset() override;

    // get filter size
    uint8_t get_filter_size() const {
        return FILTER_SIZE;
    };

    T get_sample(uint8_t i) const {
        return samples[i];
    }

protected:
    T               samples[FILTER_SIZE];       // buffer of samples
    uint8_t         sample_index;               // pointer to the next empty slot in the buffer
};

// Typedef for convenience
typedef FilterWithBuffer<int16_t,2> FilterWithBufferInt16_Size2;
typedef FilterWithBuffer<int16_t,3> FilterWithBufferInt16_Size3;
typedef FilterWithBuffer<int16_t,4> FilterWithBufferInt16_Size4;
typedef FilterWithBuffer<int16_t,5> FilterWithBufferInt16_Size5;
typedef FilterWithBuffer<int16_t,6> FilterWithBufferInt16_Size6;
typedef FilterWithBuffer<int16_t,7> FilterWithBufferInt16_Size7;
typedef FilterWithBuffer<uint16_t,2> FilterWithBufferUInt16_Size2;
typedef FilterWithBuffer<uint16_t,3> FilterWithBufferUInt16_Size3;
typedef FilterWithBuffer<uint16_t,4> FilterWithBufferUInt16_Size4;
typedef FilterWithBuffer<uint16_t,5> FilterWithBufferUInt16_Size5;
typedef FilterWithBuffer<uint16_t,6> FilterWithBufferUInt16_Size6;
typedef FilterWithBuffer<uint16_t,7> FilterWithBufferUInt16_Size7;

typedef FilterWithBuffer<int32_t,2> FilterWithBufferInt32_Size2;
typedef FilterWithBuffer<int32_t,3> FilterWithBufferInt32_Size3;
typedef FilterWithBuffer<int32_t,4> FilterWithBufferInt32_Size4;
typedef FilterWithBuffer<int32_t,5> FilterWithBufferInt32_Size5;
typedef FilterWithBuffer<int32_t,6> FilterWithBufferInt32_Size6;
typedef FilterWithBuffer<int32_t,7> FilterWithBufferInt32_Size7;
typedef FilterWithBuffer<uint32_t,2> FilterWithBufferUInt32_Size2;
typedef FilterWithBuffer<uint32_t,3> FilterWithBufferUInt32_Size3;
typedef FilterWithBuffer<uint32_t,4> FilterWithBufferUInt32_Size4;
typedef FilterWithBuffer<uint32_t,5> FilterWithBufferUInt32_Size5;
typedef FilterWithBuffer<uint32_t,6> FilterWithBufferUInt32_Size6;
typedef FilterWithBuffer<uint32_t,7> FilterWithBufferUInt32_Size7;

// Constructor
template <class T, uint8_t FILTER_SIZE>
FilterWithBuffer<T,FILTER_SIZE>::FilterWithBuffer() :
    sample_index(0)
{
    // clear sample buffer
    reset();
}

// reset - clear all samples from the buffer
template <class T, uint8_t FILTER_SIZE>
void FilterWithBuffer<T,FILTER_SIZE>::reset()
{
    // clear samples buffer
    for( int8_t i=0; i<FILTER_SIZE; i++ ) {
        samples[i] = 0;
    }

    // reset index back to beginning of the array
    sample_index = 0;
}

// apply - take in a new raw sample, and return the filtered results
template <class T, uint8_t FILTER_SIZE>
T FilterWithBuffer<T,FILTER_SIZE>::        apply(T sample)
{
    // add sample to array
    samples[sample_index++] = sample;

    // wrap index if necessary
    if( sample_index >= FILTER_SIZE )
        sample_index = 0;

    // base class doesn't know what filtering to do so we just return the raw sample
    return sample;
}
