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
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_ChibiOS_Namespace.h"

class ChibiOS::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    virtual bool give() override;
    virtual bool take(uint32_t timeout_ms) override;
    virtual bool take_nonblocking() override;

    // methods within HAL_ChibiOS only
    bool check_owner(void);
    void assert_owner(void);
protected:
    // to avoid polluting the global namespace with the 'ch' variable,
    // we declare the lock as a uint32_t array, and cast inside the cpp file
    uint32_t _lock[5];
};

/*
  BinarySemaphore implementation
 */
class ChibiOS::BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state=false);

    CLASS_NO_COPY(BinarySemaphore);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking(void) override;
    void signal(void) override;
    void signal_ISR(void) override;

protected:
    uint32_t _lock[5];
};
