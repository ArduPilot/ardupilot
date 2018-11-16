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

#pragma once

#include <uavcan/driver/system_clock.hpp>

namespace ChibiOS_CAN {

namespace clock {
/**
 * Starts the clock.
 * Can be called multiple times, only the first call will be effective.
 */
void init();

/**
 * Returns current monotonic time since the moment when clock::init() was called.
 * This function is thread safe.
 */
uavcan::MonotonicTime getMonotonic();

/**
 * Sets the driver's notion of the system UTC. It should be called
 * at startup and any time the system clock is updated from an
 * external source that is not the UAVCAN Timesync master.
 * This function is thread safe.
 */
void setUtc(uavcan::UtcTime time);

/**
 * Returns UTC time if it has been set, otherwise returns zero time.
 * This function is thread safe.
 */
uavcan::UtcTime getUtc();

/**
 * Performs UTC phase and frequency adjustment.
 * The UTC time will be zero until first adjustment has been performed.
 * This function is thread safe.
 */
void adjustUtc(uavcan::UtcDuration adjustment);

/**
 * UTC clock synchronization parameters
 */
struct UtcSyncParams {
    float offset_p;                        ///< PPM per one usec error
    float rate_i;                          ///< PPM per one PPM error for second
    float rate_error_corner_freq;
    float max_rate_correction_ppm;
    float lock_thres_rate_ppm;
    uavcan::UtcDuration lock_thres_offset;
    uavcan::UtcDuration min_jump;          ///< Min error to jump rather than change rate

    UtcSyncParams()
        : offset_p(0.01F)
        , rate_i(0.02F)
        , rate_error_corner_freq(0.01F)
        , max_rate_correction_ppm(300.0F)
        , lock_thres_rate_ppm(2.0F)
        , lock_thres_offset(uavcan::UtcDuration::fromMSec(4))
        , min_jump(uavcan::UtcDuration::fromMSec(10))
    { }
};

/**
 * Clock rate error.
 * Positive if the hardware timer is slower than reference time.
 * This function is thread safe.
 */
float getUtcRateCorrectionPPM();

/**
 * Number of non-gradual adjustments performed so far.
 * Ideally should be zero.
 * This function is thread safe.
 */
uavcan::uint32_t getUtcJumpCount();

/**
 * Whether UTC is synchronized and locked.
 * This function is thread safe.
 */
bool isUtcLocked();

/**
 * UTC sync params get/set.
 * Both functions are thread safe.
 */
UtcSyncParams getUtcSyncParams();
void setUtcSyncParams(const UtcSyncParams& params);

}

/**
 * Adapter for uavcan::ISystemClock.
 */
class SystemClock : public uavcan::ISystemClock, uavcan::Noncopyable {
    SystemClock() { }

    virtual void adjustUtc(uavcan::UtcDuration adjustment)
    {
        clock::adjustUtc(adjustment);
    }

public:
    virtual uavcan::MonotonicTime getMonotonic() const
    {
        return clock::getMonotonic();
    }
    virtual uavcan::UtcTime getUtc()             const
    {
        return clock::getUtc();
    }

    /**
     * Calls clock::init() as needed.
     * This function is thread safe.
     */
    static SystemClock& instance();
};

}
