/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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
 * Modified for Ardupilot by Siddharth Bharat Purohit
 */

#include "CANThread.h"
#include "CANClock.h"
#include "CANIface.h"
#include "CANInternal.h"


namespace ChibiOS_CAN {
/*
 * BusEvent
 */
bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    // set maximum time to allow for 16 bit timers running at 1MHz
    static const uavcan::int64_t MaxDelayUSec = 0x000FFFF;

    const uavcan::int64_t usec = duration.toUSec();
    msg_t ret = msg_t();

    if (usec <= 0) {
# if (CH_KERNEL_MAJOR == 2)
        ret = sem_.waitTimeout(TIME_IMMEDIATE);
# else // ChibiOS 3+
        ret = sem_.wait(TIME_IMMEDIATE);
# endif
    }
    else {
# if (CH_KERNEL_MAJOR == 2)
        ret = sem_.waitTimeout((usec > MaxDelayUSec) ? US2ST(MaxDelayUSec) : US2ST(usec));
# elif (CH_KERNEL_MAJOR >= 5)
        ret = sem_.wait((usec > MaxDelayUSec) ? chTimeUS2I(MaxDelayUSec) : chTimeUS2I(usec));
# else // ChibiOS 3+
        ret = sem_.wait((usec > MaxDelayUSec) ? US2ST(MaxDelayUSec) : US2ST(usec));
# endif
    }
# if (CH_KERNEL_MAJOR == 2)
    return ret == RDY_OK;
# else // ChibiOS 3+
    return ret == MSG_OK;
# endif
}

void BusEvent::signal()
{
    sem_.signal();
}

void BusEvent::signalFromInterrupt()
{
# if (CH_KERNEL_MAJOR == 2)
    chSysLockFromIsr();
    sem_.signalI();
    chSysUnlockFromIsr();
# else // ChibiOS 3+
    chSysLockFromISR();
    sem_.signalI();
    chSysUnlockFromISR();
# endif
}

/*
 * Mutex
 */
void Mutex::lock()
{
    mtx_.lock();
}

void Mutex::unlock()
{
# if (CH_KERNEL_MAJOR == 2)
    chibios_rt::BaseThread::unlockMutex();
# else // ChibiOS 3+
    mtx_.unlock();
# endif
}

}
