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

class Pollable {
public:
    Pollable(int fd) : _fd(fd) { }
    virtual ~Pollable();

    int get_fd() const { return _fd; }

    bool set_blocking(bool setting);

    virtual void on_can_read(int max_time_ms) {}
    virtual void on_can_write(int max_time_ms) {}
    virtual void on_error(int max_time_ms) {}
    virtual void on_hang_up(int max_time_ms) {}

protected:
    int _fd;
};

class BufferedPollable : public Pollable {
public:
    BufferedPollable(int fd) : Pollable(fd) { }
    BufferedPollable(int fd, uint32_t read_buffer_size, uint32_t write_buffer_size)
        : Pollable(fd)
        , _read_buffer{read_buffer_size}
        , _write_buffer{write_buffer_size} { }

    bool is_write_buffer_empty();
    uint32_t get_read_buffer_available();
    uint32_t get_write_buffer_available();

    uint32_t buffered_read(uint8_t *buf, uint32_t len);
    uint32_t buffered_write(const uint8_t *buf, uint32_t len);

    void set_blocking_writes(bool setting) { _blocking_writes = setting; }
    void set_blocking_reads(bool setting) { _blocking_reads = setting; }

    virtual void on_can_read(int max_time_ms) override;
    virtual void on_can_write(int max_time_ms) override;
    virtual void on_hang_up(int max_time_ms) override;

protected:
    ByteBuffer _read_buffer{1024}, _write_buffer{1024};
    Linux::Semaphore _write_sem, _read_sem;

    void write_fd(uint32_t n_bytes);

private:
    bool _blocking_writes{false}, _blocking_reads{false};
};

class PacketedBufferedPollable : public BufferedPollable {
private:
    uint32_t to_mavlink_boundary(uint32_t available);

public:
    PacketedBufferedPollable(int fd) : BufferedPollable(fd) { }

    virtual void on_can_write(int max_time_ms) override;
};

class Poller {
public:
    Poller() : _epfd(epoll_create1(EPOLL_CLOEXEC)) { }
    ~Poller() { close(_epfd); }

    enum Event : uint8_t {
        Read = 1<<0,
        Write = 1<<1,
        Error = 1<<2,
        All = Read | Write | Error
    };

    bool register_pollable(Pollable*, const Event);
    void unregister_pollable(const Pollable*);

    int poll(int timeout_ms) const;

private:
    int _epfd;

    static uint32_t to_epoll_events(Event events);
};

}
