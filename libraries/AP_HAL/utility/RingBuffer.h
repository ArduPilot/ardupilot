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

    // number of bytes available to be read
    uint32_t available(void) const;

    // number of bytes space available to write
    uint32_t space(void) const;

    // true if available() is zero
    bool empty(void) const;

    // write bytes to ringbuffer. Returns number of bytes written
    uint32_t write(const uint8_t *data, uint32_t len);

    // read bytes from ringbuffer. Returns number of bytes read
    uint32_t read(uint8_t *data, uint32_t len);

    /*
      update bytes at the read pointer. Used to update an object without
      popping it
    */
    bool update(const uint8_t *data, uint32_t len);
    
    // return size of ringbuffer
    uint32_t get_size(void) const { return size; }

    // set size of ringbuffer, caller responsible for locking
    void set_size(uint32_t size);
    
    // advance the read pointer (discarding bytes)
    bool advance(uint32_t n);

    // return a pointer to the next available data
    const uint8_t *readptr(uint32_t &available_bytes);

    // peek one byte without advancing read pointer. Return byte
    // or -1 if none available
    int16_t peek(uint32_t ofs) const;

    /*
      read len bytes without advancing the read pointer
    */
    uint32_t peekbytes(uint8_t *data, uint32_t len);
    
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
        buffer = new ByteBuffer((_size * sizeof(T))+1);
    }
    ~ObjectBuffer(void) {
        delete buffer;
    }

    // return number of objects available to be read
    uint32_t available(void) const {
        return buffer->available() / sizeof(T);
    }

    // return number of objects that could be written
    uint32_t space(void) const {
        return buffer->space() / sizeof(T);
    }

    // true is available() == 0
    bool empty(void) const {
        return buffer->empty();
    }

    // push one object
    bool push(const T &object) {
        if (buffer->space() < sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }
   
    /*
      throw away an object
     */
    bool pop(void) {
        return buffer->advance(sizeof(T));
    }

    /*
      pop earliest object off the queue
     */
    bool pop(T &object) {
        if (buffer->available() < sizeof(T)) {
            return false;
        }
        return buffer->read((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    
    /*
     * push_force() is semantically equivalent to:
     *   if (!push(t)) { pop(); push(t); }
     */
    bool push_force(const T &object) {
        if (buffer->space() < sizeof(T)) {
            buffer->advance(sizeof(T));
        }
        return push(object);
    }

    /*
      peek copies an object out without advancing the read pointer
     */
    bool peek(T &object) {
        return buffer->peekbytes((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    /* update the object at the front of the queue (the one that would
       be fetched by pop()) */
    bool update(const T &object) {
        return buffer->update((uint8_t*)&object, sizeof(T));
    }
    
private:
    ByteBuffer *buffer = nullptr;
};
    


/*
  ring buffer class for objects of fixed size with pointer
  access. Note that this is not thread safe, buf offers efficient
  array-like access
 */
template <class T>
class ObjectArray {
public:
    ObjectArray(uint16_t _size) {
        size = _size;
        head = count = 0;
        buffer = new T[size];
    }
    ~ObjectArray(void) {
        delete[] buffer;
    }

    // return number of objects available to be read
    uint16_t available(void) const {
        return count;
    }

    // return number of objects that could be written
    uint16_t space(void) const {
        return size - count;
    }

    // true is available() == 0
    bool empty(void) const {
        return count == 0;
    }

    // push one object
    bool push(const T &object) {
        if (space() == 0) {
            return false;
        }
        buffer[(head+count)%size] = object;
        count++;
        return true;
    }
   
    /*
      throw away an object
     */
    bool pop(void) {
        if (empty()) {
            return false;
        }
        head = (head+1) % size;
        count--;
        return true;
    }

    /*
      pop earliest object off the queue
     */
    bool pop(T &object) {
        if (empty()) {
            return false;
        }
        object = buffer[head];
        return pop();
    }

    
    /*
     * push_force() is semantically equivalent to:
     *   if (!push(t)) { pop(); push(t); }
     */
    bool push_force(const T &object) {
        if (space() == 0) {
            pop();
        }
        return push(object);
    }

    /*
      remove the Nth element from the array. First element is zero
     */
    bool remove(uint16_t n) {
        if (n >= count) {
            return false;
        }
        if (n == count-1) {
            // remove last element
            count--;
            return true;
        }
        if (n == 0) {
            // remove first element
            return pop();
        }
        // take advantage of the [] operator for simple shift of the array elements
        for (uint16_t i=n; i<count-1; i++) {
            *(*this)[i] = *(*this)[i+1];
        }
        count--;
        return true;
    }
    
    // allow array indexing, based on current head. Returns a pointer
    // to the object or NULL
    T * operator[](uint16_t i) {
        if (i >= count) {
            return nullptr;
        }
        return &buffer[(head+i)%size];
    }
    
private:
    T *buffer;
    uint16_t size;  // total buffer size
    uint16_t count; // number in buffer now
    uint16_t head;  // first element
};
    
