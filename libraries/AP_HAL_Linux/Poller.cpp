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
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include "Poller.h"

extern const AP_HAL::HAL &hal;

namespace Linux {

void WakeupPollable::on_can_read()
{
    ssize_t r;
    uint64_t val;

    do {
        r = read(_fd, &val, sizeof(val));
    } while (!(r == -1 && errno == EAGAIN));
}

Poller::Poller()
{
    _epfd = epoll_create1(EPOLL_CLOEXEC);
    if (_epfd == -1) {
        fprintf(stderr, "Failed to create epoll: %m\n");
        return;
    }

    _wakeup._fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (_wakeup._fd == -1) {
        fprintf(stderr, "Failed to create wakeup fd: %m\n");
        goto fail_eventfd;
    }

    if (!register_pollable(&_wakeup, EPOLLIN)) {
        fprintf(stderr, "Failed to add wakeup fd\n");
        goto fail_register;
    }

    return;

fail_register:
    close(_wakeup._fd);
    _wakeup._fd = -1;
fail_eventfd:
    close(_epfd);
    _epfd = -1;
}

bool Poller::register_pollable(Pollable *p, uint32_t events)
{
    /*
     * EPOLLWAKEUP prevents the system from hibernating or suspending when
     * inside epoll_wait() for this particular event.  It is silently
     * ignored if the process does not have the CAP_BLOCK_SUSPEND
     * capability.
     */
    events |= EPOLLWAKEUP;

    if (_epfd < 0) {
        return false;
    }

    struct epoll_event epev = { };
    epev.events = events;
    epev.data.ptr = static_cast<void *>(p);

    return epoll_ctl(_epfd, EPOLL_CTL_ADD, p->get_fd(), &epev) == 0;
}

void Poller::unregister_pollable(const Pollable *p)
{
    if (_epfd >= 0 && p->get_fd() >= 0) {
        epoll_ctl(_epfd, EPOLL_CTL_DEL, p->get_fd(), nullptr);
    }
}

int Poller::poll() const
{
    const int max_events = 16;
    epoll_event events[max_events];
    int r;

    do {
        r = epoll_wait(_epfd, events, max_events, -1);
    } while (r < 0 && errno == EINTR);

    if (r < 0) {
        return -errno;
    }

    for (int i = 0; i < r; i++) {
        Pollable *p = static_cast<Pollable *>(events[i].data.ptr);

        if (events[i].events & EPOLLIN) {
            p->on_can_read();
        }
        if (events[i].events & EPOLLOUT) {
            p->on_can_write();
        }
        if (events[i].events & EPOLLERR) {
            p->on_error();
        }
        if (events[i].events & EPOLLHUP) {
            p->on_hang_up();
        }
    }

    return r;
}

void Poller::wakeup() const
{
    ssize_t r;
    uint64_t val = 1;

    do {
        r = write(_wakeup.get_fd(), &val, sizeof(val));
    } while (r == -1 && errno == EINTR);

    if (r == -1) {
        fprintf(stderr, "Failed to wakeup poller: %m\n");
    }
}

Pollable::~Pollable()
{
    /*
     * Make sure to remove the file descriptor from epoll since events could
     * continue to be reported if the file descriptor was dup()'ed.  However
     * we rely on user unregistering it rather than taking a reference to the
     * Poller.  Here we just close our file descriptor.
     */
    if (_fd >= 0) {
        close(_fd);
    }
}

}
