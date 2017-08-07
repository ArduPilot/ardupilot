#include <stdlib.h>
#include <string.h>
#include <math.h>

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
    head = written =  tail = 0;
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
	return written;
}

void ByteBuffer::clear(void)
{
    head = written = tail = 0;
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
	
	uint32_t written_ = written;

    if (len > written_) {
        len = written_;
    }

    uint32_t head_to_end = size - head;
	uint32_t continious_read = head_to_end > written_ ? written_ : head_to_end;

    iovec[0].data = buf + head;
    iovec[0].len = len > continious_read ? continious_read : len;

    if (len <= iovec[0].len) {
        return 1;
    }

    iovec[1].data = buf;
    iovec[1].len = len - iovec[0].len;

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
	uint32_t written_ = written;
	uint32_t n = size - written_;

    if (len > n) {
        len = n;
    }

    if (!len) {
        return 0;
    }

	//local head copy to protect against race condition
	uint32_t head_ = head;

	uint32_t offs = 1;
	if(written_ == 0 && tail == 0) {
		offs = 0;
	}

    uint32_t tail_to_end_space = size - tail - 1;

    iovec[0].data = buf + ((tail + offs) % size);
    if (head_ == 0 || head_ > (tail+1) || tail_to_end_space > len) {
        iovec[0].len = len;
        return 1;
    }

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

	uint32_t offs = 0;
	//special case that should pretty much only occur right after
	//construction
	if(written == 0 && head == tail) {
		offs = -1;
	}
    written += len;
	tail = (tail + len + offs) % size;
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
    available_bytes = (head + _written) >= size ? size - head : (uint32_t) _written;

    return available_bytes ? &buf[head] : nullptr;
}

int16_t ByteBuffer::peek(uint32_t ofs) const
{
    if (ofs >= available()) {
        return -1;
    }
    return buf[(head+ofs)%size];
}
