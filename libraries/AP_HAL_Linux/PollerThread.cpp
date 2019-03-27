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
#include "PollerThread.h"

#include <algorithm>
#include <poll.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>

#include <AP_Math/AP_Math.h>

namespace Linux {

void TimerPollable::on_can_read()
{
    if (_removeme) {
        return;
    }

    uint64_t nevents = 0;
    int r = read(_fd, &nevents, sizeof(nevents));
    if (r < 0) {
        return;
    }

    if (_wrapper) {
        _wrapper->start_cb();
    }

    _cb();

    if (_wrapper) {
        _wrapper->end_cb();
    }
}

bool TimerPollable::setup_timer(uint32_t timeout_usec)
{
    if (_fd >= 0) {
        return false;
    }

    _fd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC|TFD_NONBLOCK);
    if (_fd < 0) {
        return false;
    }

    if (!adjust_timer(timeout_usec)) {
        ::close(_fd);
        _fd = -1;
        return false;
    }

    return true;
}

bool TimerPollable::adjust_timer(uint32_t timeout_usec)
{
    if (_fd < 0) {
        return false;
    }

    struct itimerspec spec = { };

    spec.it_interval.tv_nsec = timeout_usec * AP_NSEC_PER_USEC;
    spec.it_value.tv_nsec = timeout_usec * AP_NSEC_PER_USEC;

    if (timerfd_settime(_fd, 0, &spec, nullptr) < 0) {
        return false;
    }

    return true;
}

TimerPollable *PollerThread::add_timer(TimerPollable::PeriodicCb cb,
                                       TimerPollable::WrapperCb *wrapper,
                                       uint32_t timeout_usec)
{
    if (!_poller) {
        return nullptr;
    }
    TimerPollable *p = new TimerPollable(cb, wrapper);
    if (!p || !p->setup_timer(timeout_usec) ||
        !_poller.register_pollable(p, POLLIN)) {
        delete p;
        return nullptr;
    }

    _timers.push_back(p);

    return p;
}

bool PollerThread::adjust_timer(TimerPollable *p, uint32_t timeout_usec)
{
    /* Make sure the handle points to a valid timer */
    auto it = std::find(_timers.begin(), _timers.end(), p);
    if (it == _timers.end()) {
        return false;
    }

    return (*it)->adjust_timer(timeout_usec);
}

void PollerThread::_cleanup_timers()
{
    if (!_poller) {
        return;
    }

    for (auto it = _timers.begin(); it != _timers.end(); it++) {
        TimerPollable *p = *it;
        if (p->_removeme) {
            _timers.erase(it);
            _poller.unregister_pollable(p);
            delete p;
        }
    }
}

void PollerThread::mainloop()
{
    if (!_poller) {
        return;
    }

    while (!_should_exit) {
        _poller.poll();
        _cleanup_timers();
    }

    _started = false;
    _should_exit = false;
}

bool PollerThread::stop()
{
    if (!is_started()) {
        return false;
    }

    _should_exit = true;
    _poller.wakeup();

    return true;
}

}
