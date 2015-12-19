#include "RingBuffer.h"
#include <stdlib.h>
#include <string.h>

/*
  implement a simple ringbuffer of bytes
 */

ByteBuffer::ByteBuffer(uint32_t _size)
{
    size = _size;
    buf = new uint8_t[size];
    head = tail = 0;
}

ByteBuffer::~ByteBuffer(void)
{
    delete [] buf;
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
    if (len > space()) {
        len = space();
    }
    if (len == 0) {
        return 0;
    }
    if (tail+len <= size) {
        // perform as single memcpy
        memcpy(&buf[tail], data, len);
        tail = (tail + len) % size;
        return len;
    }

    // perform as two memcpy calls
    uint32_t n = size - tail;
    if (n > len) {
        n = len;
    }
    memcpy(&buf[tail], data, n);
    tail = (tail + n) % size;
    data += n;
    n = len - n;
    if (n > 0) {
        memcpy(&buf[tail], data, n);
        tail = (tail + n) % size;
    }
    return len;
}

bool ByteBuffer::advance(uint32_t n)
{
    if (n > available()) {
        return false;
    }
    head = (head + n) % size;
    return true;
}

uint32_t ByteBuffer::read(uint8_t *data, uint32_t len)
{
    if (len > available()) {
        len = available();
    }
    if (len == 0) {
        return 0;
    }
    uint32_t n;
    const uint8_t *b = readptr(n);
    if (n > len) {
        n = len;
    }

    // perform first memcpy
    memcpy(data, b, n);
    advance(n);
    data += n;

    if (len > n) {
        // possible second memcpy
        uint32_t n2;
        b = readptr(n2);
        if (n2 > len-n) {
            n2 = len-n;
        }
        memcpy(data, b, n2);
        advance(n2);
    }
    return len;
}

/*
  return a pointer to a contiguous read buffer
 */
const uint8_t *ByteBuffer::readptr(uint32_t &available_bytes)
{
    available_bytes = available();
    if (available_bytes == 0) {
        return nullptr;
    }
    if (head+available_bytes > size) {
        available_bytes = size - head;
    }
    return &buf[head];
}

int16_t ByteBuffer::peek(uint32_t ofs) const
{
    if (ofs >= available()) {
        return -1;
    }
    return buf[(head+ofs)%size];
}
