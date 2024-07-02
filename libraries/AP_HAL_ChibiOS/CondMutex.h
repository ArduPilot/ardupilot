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

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/CondMutex.h>
#include "AP_HAL_ChibiOS_Namespace.h"

class ChibiOS::CondMutex : public AP_HAL::CondMutex {
public:
    CondMutex();
    void lock_and_wait(AP_HAL::CondMutex::Condition condition) override;
    void lock_and_signal() override;
    void unlock() override;

protected:
    // to avoid polluting the global namespace with the 'ch' variable,
    // we declare the variables as a uint32_t arrays, and cast inside the cpp file
    uint32_t _lock[5];
    uint32_t _cond[5];
};
