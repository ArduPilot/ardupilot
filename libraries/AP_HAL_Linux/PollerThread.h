/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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

#include <inttypes.h>
#include <vector>

#include <AP_HAL/Device.h>

#include "Poller.h"
#include "Thread.h"

namespace Linux {

class TimerPollable : public Pollable {
    friend class PollerThread;

public:
    class WrapperCb {
    public:
        virtual ~WrapperCb() { }

        virtual void start_cb() { }
        virtual void end_cb() { }
    };

    using PeriodicCb = AP_HAL::Device::PeriodicCb;

    virtual ~TimerPollable() { }

    void on_can_read() override;

    bool setup_timer(uint32_t timeout_usec);
    bool adjust_timer(uint32_t timeout_usec);

protected:
    TimerPollable(PeriodicCb cb, WrapperCb *wrapper)
        : _cb(cb)
        , _wrapper(wrapper)
    {
    }

    PeriodicCb _cb;
    WrapperCb *_wrapper;
    bool _removeme;
};


class PollerThread : public Thread {
public:
    PollerThread() : Thread{FUNCTOR_BIND_MEMBER(&PollerThread::mainloop, void)} { }
    virtual ~PollerThread() { }

    TimerPollable *add_timer(TimerPollable::PeriodicCb cb,
                             TimerPollable::WrapperCb *wrapper,
                             uint32_t timeout_usec);
    bool adjust_timer(TimerPollable *p, uint32_t timeout_usec);

    void mainloop();

protected:
    void _cleanup_timers();

    Poller _poller{};
    std::vector<TimerPollable*> _timers;
};

}
