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
  expanding string for easy construction of text buffers
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

class ExpandingString {
public:

    const char *get_string(void) const {
        return buf;
    }
    uint32_t get_length(void) const {
        return used;
    }

    // print into the string
    void printf(const char *format, ...) FMT_PRINTF(2,3);

    // append data to the string
    void append(const char *s, uint32_t len);

    // destructor
    ~ExpandingString();

    bool has_failed_allocation() const {
        return allocation_failed;
    }

private:
    char *buf;
    uint32_t buflen;
    uint32_t used;
    bool allocation_failed;

    // try to expand the buffer
    bool expand(uint32_t min_needed) WARN_IF_UNUSED;
};
