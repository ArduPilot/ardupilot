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
 * Author: Siddharth Bharat Purohit
 */


#pragma once

#include <uavcan/uavcan.hpp>

namespace uavcan
{
class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable
{
public:
    SystemClock() = default;

    void adjustUtc(uavcan::UtcDuration adjustment) override
    {
        utc_adjustment_usec = adjustment.toUSec();
    }

    uavcan::MonotonicTime getMonotonic() const override
    {
        return uavcan::MonotonicTime::fromUSec(AP_HAL::native_micros64());
    }

    uavcan::UtcTime getUtc() const override
    {
        return uavcan::UtcTime::fromUSec(AP_HAL::native_micros64() + utc_adjustment_usec);
    }

    int64_t getAdjustUsec() const
    {
        return utc_adjustment_usec;
    }

    static SystemClock& instance()
    {
        static SystemClock inst;
        return inst;
    }

private:
    int64_t utc_adjustment_usec;
};
}