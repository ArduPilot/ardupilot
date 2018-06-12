/*
 * Copyright (C) Siddharth Bharat Purohit 2017
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
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif
bool ppm_init(uint32_t freq, bool active_high);
uint16_t ppm_read(uint8_t chan);

uint8_t ppm_read_bulk(uint16_t periods[], uint8_t len);
bool ppm_available();
#ifdef __cplusplus
}
#endif
