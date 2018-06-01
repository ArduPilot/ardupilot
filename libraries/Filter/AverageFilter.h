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
/// @file	AverageFilter.h
/// @brief	A class to provide the average of a number of samples
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <U> is a larger data type used during summation to prevent overflows
// 3rd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, class U, uint8_t FILTER_SIZE>
class AverageFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    // constructor
    AverageFilter() : FilterWithBuffer<T,FILTER_SIZE>(), _num_samples(0) {
    };

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T        apply(T sample);

    // reset - clear the filter
    virtual void        reset();

protected:
    // the number of samples in the filter, maxes out at size of the filter
    uint8_t        _num_samples;
};

// Typedef for convenience (1st argument is the data type, 2nd is a larger datatype to handle overflows, 3rd is buffer size)
typedef AverageFilter<int8_t,int16_t,2> AverageFilterInt8_Size2;
typedef AverageFilter<int8_t,int16_t,3> AverageFilterInt8_Size3;
typedef AverageFilter<int8_t,int16_t,4> AverageFilterInt8_Size4;
typedef AverageFilter<int8_t,int16_t,5> AverageFilterInt8_Size5;
typedef AverageFilter<uint8_t,uint16_t,2> AverageFilterUInt8_Size2;
typedef AverageFilter<uint8_t,uint16_t,3> AverageFilterUInt8_Size3;
typedef AverageFilter<uint8_t,uint16_t,4> AverageFilterUInt8_Size4;
typedef AverageFilter<uint8_t,uint16_t,5> AverageFilterUInt8_Size5;

typedef AverageFilter<int16_t,int32_t,2> AverageFilterInt16_Size2;
typedef AverageFilter<int16_t,int32_t,3> AverageFilterInt16_Size3;
typedef AverageFilter<int16_t,int32_t,4> AverageFilterInt16_Size4;
typedef AverageFilter<int16_t,int32_t,5> AverageFilterInt16_Size5;
typedef AverageFilter<uint16_t,uint32_t,2> AverageFilterUInt16_Size2;
typedef AverageFilter<uint16_t,uint32_t,3> AverageFilterUInt16_Size3;
typedef AverageFilter<uint16_t,uint32_t,4> AverageFilterUInt16_Size4;
typedef AverageFilter<uint16_t,uint32_t,5> AverageFilterUInt16_Size5;

typedef AverageFilter<int32_t,float,2> AverageFilterInt32_Size2;
typedef AverageFilter<int32_t,float,3> AverageFilterInt32_Size3;
typedef AverageFilter<int32_t,float,4> AverageFilterInt32_Size4;
typedef AverageFilter<int32_t,float,5> AverageFilterInt32_Size5;
typedef AverageFilter<uint32_t,float,2> AverageFilterUInt32_Size2;
typedef AverageFilter<uint32_t,float,3> AverageFilterUInt32_Size3;
typedef AverageFilter<uint32_t,float,4> AverageFilterUInt32_Size4;
typedef AverageFilter<uint32_t,float,5> AverageFilterUInt32_Size5;

typedef AverageFilter<float,float,5> AverageFilterFloat_Size5;

// Public Methods //////////////////////////////////////////////////////////////

template <class T, class U, uint8_t FILTER_SIZE>
T AverageFilter<T,U,FILTER_SIZE>::        apply(T sample)
{
    U        result = 0;

    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

    // increment the number of samples so far
    _num_samples++;
    if( _num_samples > FILTER_SIZE || _num_samples == 0 )
        _num_samples = FILTER_SIZE;

    // get sum of all values - there is a risk of overflow here that we ignore
    for(uint8_t i=0; i<FILTER_SIZE; i++)
        result += FilterWithBuffer<T,FILTER_SIZE>::samples[i];

    return (T)(result / _num_samples);
}

// reset - clear all samples
template <class T, class U, uint8_t FILTER_SIZE>
void AverageFilter<T,U,FILTER_SIZE>::        reset()
{
    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::reset();

    // clear our variable
    _num_samples = 0;
}

/*
 * This filter is intended to be used with integral types to be faster and
 * avoid loss of precision on floating point arithmetic. The integral type
 * chosen must be one that fits FILTER_SIZE values you are filtering.
 *
 * Differently from other average filters, the result is only returned when
 * getf()/getd() is called
 */
template <class T, class U, uint8_t FILTER_SIZE>
class AverageIntegralFilter : public AverageFilter<T,U,FILTER_SIZE>
{
public:
    /*
     * Add a new raw value to the filter: method signature is maintained from
     * AverageFilter, but it doesn't retrieve the filtered value: return value
     * is always 0. Call getf()/getd() in order to get the filtered value.
     */
    virtual T apply(T sample) override;

    // get the current value as a float
    virtual float getf();

    // get the current value as a double
    virtual double getd();
protected:
    // the current sum of samples
    U _sum = 0;
};

template <class T, class U, uint8_t FILTER_SIZE>
T AverageIntegralFilter<T,U,FILTER_SIZE>::apply(T sample)
{
    T curr = this->samples[this->sample_index];

    // call parent's parent apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

    // increment the number of samples so far
    this->_num_samples++;
    if (this->_num_samples > FILTER_SIZE || this->_num_samples == 0) {
        this->_num_samples = FILTER_SIZE;
    }

    _sum -= curr;
    _sum += sample;

    // don't return the value: caller is forced to call getf() or getd()
    return 0;
}

template <class T, class U, uint8_t FILTER_SIZE>
float AverageIntegralFilter<T,U,FILTER_SIZE>::getf()
{
    if (this->_num_samples == 0) {
        return 0.f;
    }

    return (float)_sum / this->_num_samples;
}

template <class T, class U, uint8_t FILTER_SIZE>
double AverageIntegralFilter<T,U,FILTER_SIZE>::getd()
{
    if (this->_num_samples == 0) {
        return 0.f;
    }

    return (double)_sum / this->_num_samples;
}
