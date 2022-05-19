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

#include "AP_ESC.h"

#define HAL_ESC_ENABLED 1

#if HAL_ESC_ENABLED

#include "AP_ESC_Backend.h"

extern const AP_HAL::HAL &hal;

AP_ESC_Backend::AP_ESC_Backend(AP_ESC &_frontend) : frontend(_frontend) {
}

AP_Enum<Type> AP_ESC_Backend::get_esc_type(void) const {
  return frontend.type;
}

uint32_t AP_ESC_Backend::get_esc_port(void) const {
  return frontend.serial_port_num;
}

uint32_t AP_ESC_Backend::get_esc_baud(void) const {
  return frontend.serial_port_baud;
}

#endif // HAL_ESC_ENABLED