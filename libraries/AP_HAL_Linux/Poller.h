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

#include <sys/epoll.h>
#include <unistd.h>

#include "AP_HAL/utility/RingBuffer.h"
#include "Semaphores.h"

namespace Linux {

class Poller;

class Pollable {
    friend class Poller;
public:
    Pollable(int fd) : _fd(fd) { }
    Pollable() : _fd(-1) { }

    virtual ~Pollable();

    int get_fd() const { return _fd; }

    virtual void on_can_read() { }
    virtual void on_can_write() { }
    virtual void on_error() { }
    virtual void on_hang_up() { }

protected:
    int _fd;
};

class Poller {
public:
    Poller() : _epfd(epoll_create1(EPOLL_CLOEXEC)) { }
    ~Poller() { close(_epfd); }

    bool register_pollable(Pollable*, uint32_t events);
    void unregister_pollable(const Pollable*);

    int poll() const;

private:
    int _epfd;
};

}
