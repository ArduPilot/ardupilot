#include "RingBuffer.h"
#include <stdlib.h>
#include <string.h>

/*
  implement a simple ringbuffer of bytes
 */

static inline uint32_t align_power2(uint32_t u)
{
    return 1U << ((sizeof(u) * 8) - __builtin_clz(u - 1));
}

ByteBuffer::ByteBuffer(uint32_t _size)
{
    size = align_power2(_size);
    buf = new uint8_t[size];
    head = tail = 0;
}

ByteBuffer::~ByteBuffer(void)
{
    delete [] buf;
}

/*
  caller is responsible for locking in set_size()
 */
void ByteBuffer::set_size(uint32_t _size)
{
    uint8_t *oldbuf = buf;
    head = tail = 0;
    size = align_power2(_size);
    buf = new uint8_t[size];
    delete [] oldbuf;
}

uint32_t ByteBuffer::available(void) const
{
    uint32_t _tail;
    return ((head > (_tail=tail))? (size - head) + _tail: _tail - head);
}

uint32_t ByteBuffer::space(void) const
{
    uint32_t _head;
    return (((_head=head) > tail)?(_head - tail) - 1:((size - tail) + _head) - 1);
}

bool ByteBuffer::empty(void) const
{
    return head == tail;
}

uint32_t ByteBuffer::write(const uint8_t *data, uint32_t len)
{
    ByteBuffer::IoVec vec[2];
    const auto n_vec = reserve(vec, len);
    uint32_t ret = 0;

    for (int i = 0; i < n_vec; i++) {
        memcpy(vec[i].data, data + ret, vec[i].len);
        ret += vec[i].len;
    }

    return ret;
}

/*
  update bytes at the read pointer. Used to update an object without
  popping it
 */
bool ByteBuffer::update(const uint8_t *data, uint32_t len)
{
    if (len > available()) {
        return false;
    }
    // perform as two memcpy calls
    uint32_t n = size - head;
    if (n > len) {
        n = len;
    }
    memcpy(&buf[head], data, n);
    data += n;
    if (len > n) {
        memcpy(&buf[0], data, len-n);
    }
    return true;
}

bool ByteBuffer::advance(uint32_t n)
{
    if (n > available()) {
        return false;
    }
    head = (head + n) & (size - 1);
    return true;
}

int ByteBuffer::peekiovec(ByteBuffer::IoVec iovec[2], uint32_t len)
{
    if (len > available()) {
        len = available();
    }
    if (len == 0) {
        return 0;
    }
    uint32_t n;
    auto b = readptr(n);
    if (n > len) {
        n = len;
    }

    iovec[0].data = const_cast<uint8_t *>(b);
    iovec[0].len = n;

    if (len > n) {
        iovec[1].data = buf;
        iovec[1].len = len - n;

        return 2;
    }

    return 1;
}

/*
  read len bytes without advancing the read pointer
 */
uint32_t ByteBuffer::peekbytes(uint8_t *data, uint32_t len)
{
    ByteBuffer::IoVec vec[2];
    const auto n_vec = peekiovec(vec, len);
    uint32_t ret = 0;

    for (int i = 0; i < n_vec; i++) {
        memcpy(data + ret, vec[i].data, vec[i].len);
        ret += vec[i].len;
    }

    return ret;
}

int ByteBuffer::reserve(ByteBuffer::IoVec iovec[2], uint32_t len)
{
    if (len > space()) {
        len = space();
    }
    if (!len) {
        return 0;
    }

    iovec[0].data = &buf[tail];
    if (tail+len <= size) {
        iovec[0].len = len;
    } else {
        auto n = size - tail;
        if (n > len) {
            n = len;
        }

        iovec[0].len = n;

        tail = (tail + n) & (size - 1);
        n = len - n;
        if (n > 0) {
            iovec[1].data = &buf[tail];
            iovec[1].len = n;
            return 2;
        }
    }

    return 1;
}

uint32_t ByteBuffer::read(uint8_t *data, uint32_t len)
{
    uint32_t ret = peekbytes(data, len);
    advance(ret);
    return ret;
}

/*
 * Returns the pointer and size to a contiguous read in the buffer
 */
const uint8_t *ByteBuffer::readptr(uint32_t &available_bytes)
{
    uint32_t _tail = tail;
    available_bytes = (head > _tail) ? size - head : _tail - head;

    return available_bytes ? &buf[head] : nullptr;
}

int16_t ByteBuffer::peek(uint32_t ofs) const
{
    if (ofs >= available()) {
        return -1;
    }
    return buf[(head+ofs) & (size - 1)];
}
