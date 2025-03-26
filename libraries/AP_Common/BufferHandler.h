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
 */

#pragma once

#include <AP_Common/AP_Common.h>

/// Buffer pointer handler.
/// Works as a non-owning view on a buffer ptr provided by the caller,
/// or manages the buffer lifetime on the heap when the nullptr is given.
/// @tparam T data type
/// @tparam Card cardinality, num of logical blocks in the buffer
template <class T, size_t Card = 1>
class BufferHandler
{
    const bool kAllocateHeap;
    const size_t kBlockSize;
    T *buffer;

public:

    BufferHandler(T* buf, const size_t blocksize)
        : kAllocateHeap{!buf}
        , kBlockSize{blocksize}
        , buffer{kAllocateHeap ? NEW_NOTHROW T[kBlockSize*Card] : buf}
    {
    }

    BufferHandler(const size_t blocksize) : BufferHandler(nullptr, blocksize) {}

    ~BufferHandler()
    {
        if (kAllocateHeap) {
            delete[] buffer;
        }
    }

    BufferHandler(const BufferHandler&) = delete;

    BufferHandler(BufferHandler &&o) : BufferHandler(o.buffer, o.kBlockSize)
    {
        o.buffer = nullptr;
    }

    T* ptr()    { return buffer; }

    template <size_t N>
    T* block()
    {
        if (!buffer) return nullptr;
        if (Card <= N) return nullptr;
        return buffer + (kBlockSize * N);
    }

    size_t size() const
    {
        return kBlockSize * Card;
    }
};
