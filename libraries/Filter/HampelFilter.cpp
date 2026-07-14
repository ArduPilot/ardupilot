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

#include "HampelFilter.h"
#include <AP_Common/sorting.h>
#include <string.h> //for memcopy
#include <type_traits>
template <typename T>
using accumulator_type = typename std::conditional< std::is_same<T, double>::value, double,
      typename std::conditional< std::is_same<T, float>::value, float,
      typename std::conditional< std::is_same<T, uint8_t>::value,  uint16_t,
      typename std::conditional< std::is_same<T, uint16_t>::value,  uint32_t,
      typename std::conditional< std::is_same<T, uint32_t>::value,  uint64_t,
      typename std::conditional< std::is_same<T, uint64_t>::value,  uint64_t,
      typename std::conditional< std::is_same<T, int8_t>::value,  int16_t,
      typename std::conditional< std::is_same<T, int16_t>::value,  int32_t,
      typename std::conditional< std::is_same<T, int32_t>::value,  int64_t,
      typename std::conditional< std::is_same<T, int64_t>::value,  uint64_t,
      void>::type>::type>::type>::type>::type>::type>::type>::type>::type>::type;




template <class T, uint8_t FILTER_SIZE>
HampelFilter<T, FILTER_SIZE>::HampelFilter(uint8_t first_return_element, uint8_t last_return_element) :
    FilterWithBuffer<T, FILTER_SIZE>(),
    first_output_element(first_return_element),
    last_output_element(last_return_element),
    initialized(false)
{
    static_assert(FILTER_SIZE >= 3, "Hamepel test requires at least 3 samples");

    if (first_output_element >= FILTER_SIZE) {
        first_output_element = (FILTER_SIZE - 1) / 2;
        last_output_element = FILTER_SIZE / 2;  //if first element argument didn't make sense the last element must be this to make any sense
    }
    if (last_output_element >= FILTER_SIZE) {
        last_output_element = first_output_element;
    }
}
template <class T, uint8_t FILTER_SIZE>
HampelFilter<T, FILTER_SIZE>::HampelFilter(uint8_t return_element) : HampelFilter<T, FILTER_SIZE>(return_element, return_element) {}

template <class T, uint8_t FILTER_SIZE>
HampelFilter<T, FILTER_SIZE>::HampelFilter() : HampelFilter<T, FILTER_SIZE>((FILTER_SIZE-1) / 2, FILTER_SIZE / 2) {}

template <class T, uint8_t FILTER_SIZE>
void HampelFilter<T, FILTER_SIZE>::pass_sample(T sample)
{
    output_ready = false;
    initialized = true;
    FilterWithBuffer<T, FILTER_SIZE>::apply(sample);
}

template <class T, uint8_t FILTER_SIZE>
T HampelFilter<T, FILTER_SIZE>::apply(T sample)
{
    if (!initialized) {
        initialized = true;
        for (uint8_t i = 0; i < FILTER_SIZE; i++) {
            FilterWithBuffer<T, FILTER_SIZE>::samples[i] = sample;
        }
        return output = sample;
    }
    FilterWithBuffer<T, FILTER_SIZE>::apply(sample);
    return calculate();
}

template <class T, uint8_t FILTER_SIZE>
T HampelFilter<T, FILTER_SIZE>::calculate()
{
    T buf[FILTER_SIZE];
    memcpy(buf, FilterWithBuffer<T, FILTER_SIZE>::samples, FILTER_SIZE*sizeof(T));
    insertion_sort(buf, FILTER_SIZE);
    T median = get_median(buf);
    T abs_devs[FILTER_SIZE];
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        if (buf[i] > median) {                 //handle unsigned types
            abs_devs[i] = buf[i]-median;
        } else {
            abs_devs[i] = median -buf[i];
        }
    }
    insertion_sort(abs_devs, FILTER_SIZE);
    T mad = get_median(abs_devs)*3; // multiply MAD by 3 to obtain ±2 sigma gate assuming samples are coming from normal distribution so 95.6% samples are accepted

    uint8_t adjusted_elements = 0;
    T lower_limit = median - mad;
    T upper_limit = median + mad;
    for (uint8_t i = 0; i < FILTER_SIZE / 2; i++) {
        if (buf[i] < lower_limit) {
            buf[i] = median;
            adjusted_elements++;
        } else { //buf is sorted, no need to check if further elements are below lower limit
            break;
        }
    }
    for (uint8_t i = FILTER_SIZE-1; i > (FILTER_SIZE - 1) / 2; i--) {
        if (buf[i] > upper_limit) {
            buf[i] = median;
            adjusted_elements++;
        } else { //buf is sorted in relevant range, no need to check if further (while going backwards) elements are above upper limit
            break;
        }
    }
    insertion_sort(buf, FILTER_SIZE);
    output_ready = true;
    if (first_output_element == last_output_element) {
        return output = buf [first_output_element];
    } else {
        accumulator_type<T> accumulator = 0;

        for (uint8_t i = first_output_element; i <= last_output_element; i++) {
            accumulator += buf[i];
        }
        return output = accumulator / (last_output_element - first_output_element + 1);
    }
}


template <class T, uint8_t FILTER_SIZE>
T HampelFilter<T, FILTER_SIZE>::get_median(T *buf)
{
    if (FILTER_SIZE% 2 == 1) { //should be cnstexpr in C++17
        return buf[(FILTER_SIZE - 1) / 2];
    } else {
        return (buf[(FILTER_SIZE) / 2 - 1] + buf[FILTER_SIZE / 2]) / 2;
    }
}

template class HampelFilter<float, 5>;
template class HampelFilter<int16_t, 5>;
