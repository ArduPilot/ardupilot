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

#include <AP_Common/AP_Common.h>

#include <stdint.h>

class ExpandingString {
public:
    ExpandingString() : buf(0), buflen(0), used(0), allocation_failed(false), external_buffer(false) {}
    ExpandingString(char* s, uint32_t total_len);

    const char *get_string(void) const {
        return buf;
    }
    uint32_t get_length(void) const {
        return used;
    }
    char *get_writeable_string(void) const {
        return buf;
    }

    // print into the string
    void printf(const char *format, ...) FMT_PRINTF(2,3);

    // append data to the string. s can be null for zero fill
    bool append(const char *s, uint32_t len);

    // set address to custom external buffer
    void set_buffer(char *s, uint32_t total_len, uint32_t used_len);
    // zero out the string
    void reset() { used = 0; }

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
    bool external_buffer;

    // try to expand the buffer
    bool expand(uint32_t min_needed) WARN_IF_UNUSED;
};
