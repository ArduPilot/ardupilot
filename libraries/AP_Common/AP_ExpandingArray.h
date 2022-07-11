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

/*
 * ExpandingArray class description
 *
 * ExpandingArrayGeneric implements most of the required functionality and is type agnostic allowing smaller overall code size
 * ExpandingArray<T> is the template and implements the small number of type specific methods
 *
 * Elements are organised into "chunks" with each chunk holding "chunk_size" elements
 * The "chunk_ptrs" array holds pointers to all allocated chunks
 *
 * The "expand" function allows expanding the array by a specified number of chunks
 * The "expand_to_hold" function expands the array (if necessary) to hold at least the specified number of elements
 *
 * When the array is expanded up to two memory allocations are required:
 *    1. if the chunk_ptrs array (which holds points to all allocated chunks) is full, this array will be re-allocated.
 *       During this operation a new copy of the chunk_ptr array will be created with "chunk_ptr_increment" more rows,
 *       the old array's data will be copied to the new array and finally the old array will be freed.
 *    2. a new chunk will be allocated and a pointer to this new chunk will be added to the chunk_ptrs array
 *
 * Warnings:
 *    1. memset, memcpy, memcmp cannot be used because the individual elements are not guaranteed to be next to each other in memory
 *    2. operator[] functions do not perform any range checking so max_items() should be used when necessary to avoid out-of-bound memory access
 *    3. elements_per_chunk (provided in constructor) should be a factor of 2 (i.e. 16, 32, 64) for best performance
 */

#pragma once

#include <AP_Common/AP_Common.h>

class AP_ExpandingArrayGeneric
{
public:

    AP_ExpandingArrayGeneric(uint16_t element_size, uint16_t elements_per_chunk) :
        elem_size(element_size),
        chunk_size(elements_per_chunk)
    {}

    ~AP_ExpandingArrayGeneric(void);

    /* Do not allow copies */
    AP_ExpandingArrayGeneric(const AP_ExpandingArrayGeneric &other) = delete;
    AP_ExpandingArrayGeneric &operator=(const AP_ExpandingArrayGeneric&) = delete;

    // current maximum number of items (using expand may increase this)
    uint16_t max_items() const { return chunk_size * chunk_count; }

    // expand the array by specified number of chunks, returns true on success
    bool expand(uint16_t num_chunks = 1);

    // expand to hold at least num_items
    bool expand_to_hold(uint16_t num_items);

protected:

    const uint16_t elem_size;   // number of bytes for each element
    const uint16_t chunk_size;  // the number of T elements in each chunk
    const uint16_t chunk_ptr_increment = 32;    // chunk_ptrs array is grown by this many elements each time it fills

    typedef uint8_t* chunk_ptr_t;   // pointer to a chunk

    chunk_ptr_t *chunk_ptrs;    // array of pointers to allocated chunks
    uint16_t chunk_count_max;   // number of elements in chunk_ptrs array
    uint16_t chunk_count;       // number of allocated chunks
};

template <typename T>
class AP_ExpandingArray : public AP_ExpandingArrayGeneric
{
public:

    AP_ExpandingArray(uint16_t elements_per_chunk) :
        AP_ExpandingArrayGeneric(sizeof(T), elements_per_chunk)
    {}

    /* Do not allow copies */
    AP_ExpandingArray(const AP_ExpandingArray<T> &other) = delete;
    AP_ExpandingArray<T> &operator=(const AP_ExpandingArray<T>&) = delete;

    // allow use as an array for assigning to elements. no bounds checking is performed
    T &operator[](uint16_t i)
    {
        const uint16_t chunk_num = i / chunk_size;
        const uint16_t chunk_index = (i % chunk_size);
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wcast-align"
        T *el_array = (T *)chunk_ptrs[chunk_num];
        #pragma GCC diagnostic pop
        return el_array[chunk_index];
    }

    // allow use as an array for accessing elements. no bounds checking is performed
    const T &operator[](uint16_t i) const
    {
        const uint16_t chunk_num = i / chunk_size;
        const uint16_t chunk_index = (i % chunk_size);
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wcast-align"
        const T *el_array = (const T *)chunk_ptrs[chunk_num];
        #pragma GCC diagnostic pop
        return el_array[chunk_index];
    }
};
