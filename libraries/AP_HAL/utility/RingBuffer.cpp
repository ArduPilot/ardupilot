#include <stdlib.h>
#include <string.h>

#include "RingBuffer.h"

ByteBuffer::ByteBuffer(uint32_t _size)
{
    buf = (uint8_t*)calloc(1, _size);
    size = buf ? _size : 0;
    external_buf = false;
}

ByteBuffer::~ByteBuffer(void)
{
    if (!external_buf) {
        free(buf);
    }
}

/*
 * Caller is responsible for locking in set_size()
 */
bool ByteBuffer::set_size(uint32_t _size)
{
    if (external_buf) {
        // resize not supported with external buffer
        return false;
    }
    head = tail = 0;
    if (_size != size) {
        free(buf);
        buf = (uint8_t*)calloc(1, _size);
        if (!buf) {
            size = 0;
            return false;
        }

        size = _size;
    }

    return true;
}

uint32_t ByteBuffer::available(void) const
{
    /* use a copy on stack to avoid race conditions of @tail being updated by
     * the writer thread */
    uint32_t _tail = tail;

    if (head > _tail) {
        return size - head + _tail;
    }
    return _tail - head;
}

void ByteBuffer::clear(void)
{
    head = tail = 0;
}

uint32_t ByteBuffer::space(void) const
{
    if (size == 0) {
        return 0;
    }

    /* use a copy on stack to avoid race conditions of @head being updated by
     * the reader thread */
    uint32_t _head = head;
    uint32_t ret = 0;

    if (_head <= tail) {
        ret = size;
    }

    ret += _head - tail - 1;

    return ret;
}

bool ByteBuffer::is_empty(void) const
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

    commit(ret);
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
    head = (head + n) % size;
    return true;
}

uint8_t ByteBuffer::peekiovec(ByteBuffer::IoVec iovec[2], uint32_t len)
{
    uint32_t n = available();

    if (len > n) {
        len = n;
    }
    if (len == 0) {
        return 0;
    }

    auto b = readptr(n);
    if (n == 0) {
        iovec[0].data = buf;
        iovec[0].len = len;
        iovec[1].data = nullptr;
        iovec[1].len = 0;
        return 1;
    }
    if (n > len) {
        n = len;
    }

    iovec[0].data = const_cast<uint8_t *>(b);
    iovec[0].len = n;

    if (len <= n) {
        return 1;
    }

    iovec[1].data = buf;
    iovec[1].len = len - n;

    return 2;
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

uint8_t ByteBuffer::reserve(ByteBuffer::IoVec iovec[2], uint32_t len)
{
    uint32_t n = space();

    if (len > n) {
        len = n;
    }

    if (!len) {
        return 0;
    }

    iovec[0].data = &buf[tail];

    n = size - tail;
    if (len <= n) {
        iovec[0].len = len;
        return 1;
    }

    iovec[0].len = n;

    iovec[1].data = buf;
    iovec[1].len = len - n;

    return 2;
}

/*
 * Advance the writer pointer by 'len'
 */
bool ByteBuffer::commit(uint32_t len)
{
    if (len > space()) {
        return false; //Someone broke the agreement
    }

    tail = (tail + len) % size;
    return true;
}

uint32_t ByteBuffer::read(uint8_t *data, uint32_t len)
{
    uint32_t ret = peekbytes(data, len);
    advance(ret);
    return ret;
}

bool ByteBuffer::read_byte(uint8_t *data)
{
    if (!data) {
        return false;
    }

    int16_t ret = peek(0);
    if (ret < 0) {
        return false;
    }

    *data = ret;

    return advance(1);
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
    return buf[(head+ofs)%size];
}
