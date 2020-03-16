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

#include "AP_ExpandingArray.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_ExpandingArrayGeneric::~AP_ExpandingArrayGeneric(void)
{
    // free chunks
    for (uint16_t i=0; i<chunk_count; i++) {
        free(chunk_ptrs[i]);
    }
    // free chunks_ptrs array
    free(chunk_ptrs);
}

// expand the array by specified number of chunks, returns true on success
bool AP_ExpandingArrayGeneric::expand(uint16_t num_chunks)
{
    // expand chunk_ptrs array if necessary
    if (chunk_count + num_chunks >= chunk_count_max) {
        uint16_t chunk_ptr_size = chunk_count + num_chunks + chunk_ptr_increment;
        if (hal.util->available_memory() < 100U + (chunk_ptr_size * sizeof(chunk_ptr_t))) {
            // fail if reallocating would leave less than 100 bytes of memory free
            return false;
        }
        chunk_ptr_t *chunk_ptrs_new = (chunk_ptr_t*)hal.util->std_realloc((void*)chunk_ptrs, chunk_ptr_size * sizeof(chunk_ptr_t));
        if (chunk_ptrs_new == nullptr) {
            return false;
        }

        // use new pointers array
        chunk_ptrs = chunk_ptrs_new;
        chunk_count_max = chunk_ptr_size;
    }

    // allocate new chunks
    for (uint16_t i = 0; i < num_chunks; i++) {
        if (hal.util->available_memory() < 100U + (chunk_size * elem_size)) {
            // fail if reallocating would leave less than 100 bytes of memory free
            return false;
        }
        uint8_t *new_chunk = (uint8_t *)calloc(chunk_size, elem_size);
        if (new_chunk == nullptr) {
            // failed to allocate new chunk
            return false;
        }
        chunk_ptrs[chunk_count] = new_chunk;
        chunk_count++;
    }
    return true;
}

// expand to hold at least num_items
bool AP_ExpandingArrayGeneric::expand_to_hold(uint16_t num_items)
{
    // check if already big enough
    if (num_items <= max_items()) {
        return true;
    }
    uint16_t chunks_required = ((num_items - max_items()) / chunk_size) + 1;
    return expand(chunks_required);
}
