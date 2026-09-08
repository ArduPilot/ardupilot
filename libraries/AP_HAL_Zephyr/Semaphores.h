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
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/Semaphores.h>

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#endif

namespace Zephyr {

class Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override WARN_IF_UNUSED;
    bool take_nonblocking() override WARN_IF_UNUSED;

private:
#ifdef __ZEPHYR__
    struct k_mutex _mutex;
#endif
};

class BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state = false);
    bool wait(uint32_t timeout_us) override WARN_IF_UNUSED;
    bool wait_blocking() override;
    void signal() override;
    void signal_ISR() override;

private:
#ifdef __ZEPHYR__
    struct k_sem _sem;
#endif
};

}  // namespace Zephyr
