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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS_Namespace.h>
#include "hwdef/common/halconf.h"
#ifdef USE_POSIX
#include <ff.h>
#endif
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "hrt.h"

class HAL_ChibiOS : public AP_HAL::HAL {
public:
    HAL_ChibiOS();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
void hal_chibios_set_priority(uint8_t priority);

thread_t* get_main_thread(void);
