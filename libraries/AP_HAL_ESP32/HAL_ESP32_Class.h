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

#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_ESP32/HAL_ESP32_Namespace.h>

class HAL_ESP32 : public AP_HAL::HAL {
public:
    HAL_ESP32();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
