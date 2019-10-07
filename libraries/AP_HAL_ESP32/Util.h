/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include <esp_timer.h>
#include <multi_heap.h>

//see components/heap/include/esp_heas_cap.h

class ESP32::Util : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) override { return false; }

    /**
       how much free memory do we have in bytes.  on esp32 this returns the
       largest free block of memory able to be allocated with the given capabilities.
       , which in this case is "Memory must be able to run executable code"
     */
     virtual uint32_t available_memory(void) override {
    	 return heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
     }

    uint64_t get_hw_rtc() const override
	{
		return esp_timer_get_time();
	}

#ifdef ENABLE_HEAP
    // heap functions, if all block of the heap are freed the heap area can be
	// used normally
    void *allocate_heap_memory(size_t size) override {
		void *buf = malloc(size);
		if (buf == nullptr) {
			return nullptr;
		}

		multi_heap_handle_t *heap = (multi_heap_handle_t *)malloc(sizeof(multi_heap_handle_t));
		if (heap != nullptr) {
			auto hp = multi_heap_register(buf, size);
			memcpy(heap, &hp, sizeof(multi_heap_handle_t));
		}

		return heap;
	}
    void *heap_realloc(void *heap, void *ptr, size_t new_size) override
	{
		if (heap == nullptr) {
			return nullptr;
		}

		return multi_heap_realloc(*(multi_heap_handle_t *)heap, ptr, new_size);
	}
#endif // ENABLE_HEAP


};
