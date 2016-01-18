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
  Flymaple port by Mike McCauley
 */

#ifndef __AP_HAL_FLYMAPLE_STORAGE_H__
#define __AP_HAL_FLYMAPLE_STORAGE_H__

#include <AP_HAL_FLYMAPLE.h>

class AP_HAL_FLYMAPLE_NS::FLYMAPLEStorage : public AP_HAL::Storage {
public:
    FLYMAPLEStorage();
    void init(void *);
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);

private:
    uint8_t read_byte(uint16_t loc);
    void write_byte(uint16_t loc, uint8_t value);
};

#endif // __AP_HAL_FLYMAPLE_STORAGE_H__
