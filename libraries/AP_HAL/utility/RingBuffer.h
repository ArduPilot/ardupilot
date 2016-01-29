#pragma once

#include <stdint.h>
#include <stdbool.h>


/*
  old style ring buffer handling macros

  These macros assume a ring buffer like this:

    uint8_t *_buf;
    uint16_t _buf_size;
    volatile uint16_t _buf_head;
    volatile uint16_t _buf_tail;

  These should be converted to a class in future
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size


/*
  new style buffers
 */
class ByteBuffer {
public:
    ByteBuffer(uint32_t size);
    ~ByteBuffer(void);
    
    uint32_t available(void) const;
    uint32_t space(void) const;
    bool empty(void) const;
    uint32_t write(const uint8_t *data, uint32_t len);
    uint32_t read(uint8_t *data, uint32_t len);
    uint32_t get_size(void) const { return size; }
    bool advance(uint32_t n);
    const uint8_t *readptr(uint32_t &available_bytes);
    int16_t peek(uint32_t ofs) const;
    
private:
    uint8_t *buf = nullptr;
    uint32_t size = 0;

    // head is where the next available data is. tail is where new
    // data is written
    volatile uint32_t head = 0;
    volatile uint32_t tail = 0;
};

/*
  ring buffer class for objects of fixed size
 */
template <class T>
class ObjectBuffer {
public:
    ObjectBuffer(uint32_t _size) {
        size = _size;
        buffer = new ByteBuffer((size * sizeof(T))+1);
    }
    ~ObjectBuffer(void) {
        delete buffer;
    }

    uint32_t available(void) const {
        return buffer->available() / sizeof(T);
    }
    uint32_t space(void) const {
        return buffer->space() / sizeof(T);
    }
    bool empty(void) const {
        return buffer->empty();
    }
    bool push(const T &object) {
        if (buffer->space() < sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }
    bool pop(T &object) {
        if (buffer->available() < sizeof(T)) {
            return false;
        }
        return buffer->read((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

private:
    ByteBuffer *buffer = nullptr;
    uint32_t size = 0;
};
    
