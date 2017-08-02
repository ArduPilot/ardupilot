#include <stdlib.h>
#include <string.h>

#include "RingBuffer.h"

ByteBuffer::ByteBuffer(uint32_t _size)
{
    buf = (uint8_t*)malloc(_size);
    size = buf ? _size : 0;
}

ByteBuffer::~ByteBuffer(void)
{
    free(buf);
}

/*
 * Caller is responsible for locking in set_size()
 */
bool ByteBuffer::set_size(uint32_t _size)
{
    head = written = 0;
    if (_size != size) {
        free(buf);
        buf = (uint8_t*)malloc(_size);
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
	return written;
}

void ByteBuffer::clear(void)
{
    head = written = 0;
}

uint32_t ByteBuffer::space(void) const
{
    return size - written;
}

bool ByteBuffer::empty(void) const
{
    return written == 0;
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

    if (len > n) {
        memcpy(&buf[0], data + n, len-n);
    }
    return true;
}

bool ByteBuffer::advance(uint32_t n)
{
    if (n > available()) {
        return false;
    }
    head = (head + n) % size;
	written -= n;
    return true;
}

uint8_t ByteBuffer::peekiovec(ByteBuffer::IoVec iovec[2], uint32_t len)
{
	if(empty()) {
		return 0;
	}

    uint32_t n = available();

    if (len > n) {
        len = n;
    }

    auto b = readptr(n);
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

	//local head copy to protect against race condition
	uint32_t head_ = head;
	uint32_t tail = (head + written) % size;
    iovec[0].data = buf + tail;
    if (head_ == 0 || head_ > tail) {
        iovec[0].len = len;
        return 1;
    }

    uint32_t tail_to_end_space = size - tail;
    iovec[0].len = tail_to_end_space;

    iovec[1].data = buf;
    iovec[1].len = len - tail_to_end_space;

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

    written += len;
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
    uint32_t _written = written;
    available_bytes = (head + _written) >= size ? size - head : (uint32_t) written;

    return available_bytes ? &buf[head] : nullptr;
}

int16_t ByteBuffer::peek(uint32_t ofs) const
{
    if (ofs >= available()) {
        return -1;
    }
    return buf[(head+ofs)%size];
}
