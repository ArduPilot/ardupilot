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

/*
  caller is responsible for locking in set_size()
 */
void ByteBuffer::set_size(uint32_t _size)
{
    uint8_t *oldbuf = buf;
    head = tail = 0;
    size = _size;
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

/*
  read len bytes without advancing the read pointer
 */
uint32_t ByteBuffer::peekbytes(uint8_t *data, uint32_t len)
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
    data += n;

    if (len > n) {
        // possible second memcpy, must be from front of buffer
        memcpy(data, buf, len-n);
    }
    return len;
}

uint32_t ByteBuffer::read(uint8_t *data, uint32_t len)
{
    uint32_t ret = peekbytes(data, len);
    advance(ret);
    return ret;
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
