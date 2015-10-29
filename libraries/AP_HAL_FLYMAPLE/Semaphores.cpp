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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "Semaphores.h"

using namespace AP_HAL_FLYMAPLE_NS;

#include "AP_HAL_FLYMAPLE.h"
#include "Semaphores.h"
#include "Scheduler.h"
#include "FlymapleWirish.h"

extern const AP_HAL::HAL& hal;

// Constructor
FLYMAPLESemaphore::FLYMAPLESemaphore() : _taken(false) {}

bool FLYMAPLESemaphore::give() {
    if (!_taken) {
        return false;
    } else {
        _taken = false;
        return true;
    }
}

bool FLYMAPLESemaphore::take(uint32_t timeout_ms) {
    if (hal.scheduler->in_timerprocess()) {
        hal.scheduler->panic(PSTR("PANIC: FLYMAPLESemaphore::take used from "
                    "inside timer process"));
        return false; /* Never reached - panic does not return */
    }
    return _take_from_mainloop(timeout_ms);
}

bool FLYMAPLESemaphore::take_nonblocking() {
    if (hal.scheduler->in_timerprocess()) {
        return _take_nonblocking();
    } else {
        return _take_from_mainloop(0);
    }
}

bool FLYMAPLESemaphore::_take_from_mainloop(uint32_t timeout_ms) {
    /* Try to take immediately */
    if (_take_nonblocking()) {
        return true;
    } else if (timeout_ms == 0) {
        /* Return immediately if timeout is 0 */
        return false;
    }

    do {
        /* Delay 1ms until we can successfully take, or we timed out */
        hal.scheduler->delay(1);
        timeout_ms--;
        if (_take_nonblocking()) {
            return true;
        }
    } while (timeout_ms > 0);

    return false;
}

bool FLYMAPLESemaphore::_take_nonblocking() {
    bool result = false;
    noInterrupts();
    if (!_taken) {
        _taken = true;
        result = true;
    }
    interrupts();
    return result;
}

#endif
