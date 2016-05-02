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
#include <algorithm>
#include <cstddef>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/epoll.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include "Poller.h"
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

namespace Linux {

uint32_t Poller::to_epoll_events(Poller::Event ev) {
    // EPOLLWAKEUP prevents the system from hibernating or suspending when
    // inside epoll_wait() for this particular event.  It is silently
    // ignored if the process does not have the CAP_BLOCK_SUSPEND
    // capability.
    uint32_t op = EPOLLWAKEUP;

    if (ev & Poller::Event::Read) {
        op |= EPOLLIN;
    }
    if (ev & Poller::Event::Write) {
        op |= EPOLLOUT;
    }
    if (ev & Poller::Event::Error) {
        op |= EPOLLERR;
    }

    return op;
}

bool Poller::register_pollable(Pollable *p, const Poller::Event ev) {
    if (_epfd < 0) {
        return false;
    }

    struct epoll_event epev = {0};
    epev.events = Poller::to_epoll_events(ev);
    epev.data.ptr = static_cast<void *>(p);

    return epoll_ctl(_epfd, EPOLL_CTL_ADD, p->get_fd(), &epev) == 0;
}

void Poller::unregister_pollable(const Pollable *p) {
    if (_epfd >= 0) {
        epoll_ctl(_epfd, EPOLL_CTL_DEL, p->get_fd(), NULL);
    }
}

int Poller::poll(int timeout_ms) const {
    const int max_events = 16;
    epoll_event events[max_events];
    const auto before_wait_us = AP_HAL::micros64();
    const auto r = epoll_wait(_epfd, events, max_events, timeout_ms);
    const auto delta_us = AP_HAL::micros64() - before_wait_us;
    const auto delta_ms = delta_us / 1000;
    const auto remaining_time_ms = timeout_ms - delta_ms;

    if (r > 0) {
        auto max_time_ms = remaining_time_ms / r;

        if (!max_time_ms) {
            max_time_ms = 1;
        }

        for (int i = 0; i < r; i++) {
            Pollable *p = static_cast<Pollable *>(events[i].data.ptr);

            if (events[i].events & EPOLLIN) {
                p->on_can_read(max_time_ms);
            }
            if (events[i].events & EPOLLOUT) {
                p->on_can_write(max_time_ms);
            }
            if (events[i].events & EPOLLERR) {
                p->on_error(max_time_ms);
            }
            if (events[i].events & EPOLLHUP) {
                p->on_hang_up(max_time_ms);
            }
        }
    } else if (r < 0) {
        if (errno == EINTR) {
            // Try polling again with the remaining wait time.
            return poll(remaining_time_ms);
        }
    }

    return r;
}

Pollable::~Pollable() {
    close(_fd);
}

bool Pollable::set_blocking(bool setting) {
    auto curflags = fcntl(_fd, F_GETFL, 0);

    if (curflags < 0) {
        return false;
    }

    if (setting) {
        curflags &= ~O_NONBLOCK;
    } else {
        curflags |= O_NONBLOCK;
    }

    return fcntl(_fd, F_SETFL, curflags) == 0;
}

void BufferedPollable::on_can_read(int max_time_ms) {
    if (!_read_sem.take(max_time_ms)) {
        return;
    }

    ByteBuffer::IoVec vec[2];
    const auto n_vec = _read_buffer.reserve(vec, _read_buffer.space());
    if (n_vec) {
        struct iovec iovec[n_vec];
        for (int i = 0; i < n_vec; i++) {
            iovec[i].iov_base = static_cast<void *>(vec[i].data);
            iovec[i].iov_len = static_cast<size_t>(vec[i].len);
        }

        (void) readv(_fd, iovec, n_vec);
    }

    _read_sem.give();
}

void BufferedPollable::on_hang_up(int max_time_ms) {
    auto half_time_ms = max_time_ms / 2;

    if (!_read_sem.take(half_time_ms)) {
        return;
    }

    if (_write_sem.take(half_time_ms)) {
        _read_buffer.advance(_read_buffer.available());
        _write_buffer.advance(_write_buffer.available());
        close(_fd);
        _fd = -1;

        _write_sem.give();
    }

    _read_sem.give();
}

void BufferedPollable::write_fd(uint32_t n_bytes) {
    // NOTE: Must be called with _write_sem taken.

    if (!n_bytes) {
        return;
    }

    ByteBuffer::IoVec vec[2];
    auto n_vec = _write_buffer.peekiovec(vec, n_bytes);
    if (!n_vec) {
        return;
    }

    struct iovec iovec[n_vec];
    for (int i = 0; i < n_vec; i++) {
        iovec[i].iov_base = static_cast<void *>(vec[i].data);
        iovec[i].iov_len = static_cast<size_t>(vec[i].len);
    }

    auto written = writev(_fd, iovec, n_vec);
    if (written > 0) {
        _write_buffer.advance(static_cast<uint32_t>(written));
    }
}

void BufferedPollable::on_can_write(int max_time_ms) {
    if (_write_sem.take(max_time_ms)) {
        write_fd(_write_buffer.available());
        _write_sem.give();
    }
}

bool BufferedPollable::is_write_buffer_empty() {
    bool ret = false;

    if (_write_sem.take_nonblocking()) {
        ret = _write_buffer.available() > 0;
        _write_sem.give();
    }

    return ret;
}

uint32_t BufferedPollable::get_read_buffer_available() {
    uint32_t ret = 0;

    if (_read_sem.take_nonblocking()) {
        ret = _read_buffer.available();
        _read_sem.give();
    }

    return ret;
}

uint32_t BufferedPollable::get_write_buffer_available() {
    uint32_t ret = 0;

    if (_write_sem.take_nonblocking()) {
        ret = _write_buffer.available();
        _write_sem.give();
    }

    return ret;
}

uint32_t BufferedPollable::buffered_read(uint8_t *ptr, uint32_t len) {
    uint32_t ret = 0;
    bool taken_sem;

    if (_blocking_reads) {
        taken_sem = _read_sem.take(100);
    } else {
        taken_sem = _read_sem.take_nonblocking();
    }
    if (taken_sem) {
        ret = _read_buffer.read(ptr, len);
        _read_sem.give();
    }

    return ret;
}

uint32_t BufferedPollable::buffered_write(const uint8_t *buf, uint32_t len) {
    uint32_t ret = 0;
    bool taken_sem;

    if (_blocking_writes) {
        taken_sem = _write_sem.take(100);
    } else {
        taken_sem = _write_sem.take_nonblocking();
    }
    if (taken_sem) {
        if (_write_buffer.space() >= len) {
            ret = _write_buffer.write(buf, len);
        }
        _write_sem.give();
    }

    return ret;
}

uint32_t PacketedBufferedPollable::to_mavlink_boundary(uint32_t available) {
    // NOTE: Must be called with _write_sem taken.
    const uint32_t mavlink_hdr_size = 8; // 6-byte header + 2-byte cksum.
    const uint32_t mavlink_max_size = 256;
    const uint8_t mavlink_marker = 254;

    if (!available) {
        return 0;
    }

    if (_write_buffer.peek(0) != mavlink_marker) {
        // Non-mavlink packet at the start of the buffer.  Look ahead for a
        // MAVLink start byte, up to 256 bytes ahead.
        const auto limit = std::min(mavlink_max_size, available);
        uint32_t contiguous_avail;
        const uint8_t *ptr = _write_buffer.readptr(contiguous_avail);

        if (contiguous_avail >= limit) {
            // If there's enough contiguous data in the ring buffer, use a
            // fast byte scan instead.  This should happen more often.
            auto marker_pos = memchr(ptr, mavlink_marker, limit);
            if (marker_pos) {
                return static_cast<uint32_t>(
                    static_cast<const uint8_t *>(marker_pos) - ptr);
            }
        } else {
            for (uint32_t i = 0; i < limit; i++) {
                if (_write_buffer.peek(i) == mavlink_marker) {
                    return i;
                }
            }
        }

        // No MAVLink marker, limit the send size to mavlink_max_size.
        return limit;
    }

    if (available < mavlink_hdr_size) {
        return 0; // Not a full MAVLink packet yet.
    }

    // Possible MAVLink packet, just check if it is complete.
    const auto pktlen = _write_buffer.peek(1); // Length is on 2nd byte.
    if (pktlen == -1 || available < static_cast<uint8_t>(pktlen) + mavlink_hdr_size) {
        return 0; // Not a full MAVLink packet yet.
    }

    // Packet seems complete. Send one at a time.
    return pktlen + mavlink_hdr_size;
}

void PacketedBufferedPollable::on_can_write(int max_time_ms) {
    if (_write_sem.take(max_time_ms)) {
        write_fd(to_mavlink_boundary(_write_buffer.available()));
        _write_sem.give();
    }
}

}
