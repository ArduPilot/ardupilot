#pragma once

#include <atomic>
#include <stdint.h>

/*
 * Circular buffer of bytes.
 */
class ByteBuffer {
public:
    ByteBuffer(uint32_t size);
    ~ByteBuffer(void);

    // number of bytes available to be read
    uint32_t available(void) const;

    // Discards the buffer content, emptying it.
    void clear(void);

    // number of bytes space available to write
    uint32_t space(void) const;

    // true if available() is zero
    bool empty(void) const;

    // write bytes to ringbuffer. Returns number of bytes written
    uint32_t write(const uint8_t *data, uint32_t len);

    // read bytes from ringbuffer. Returns number of bytes read
    uint32_t read(uint8_t *data, uint32_t len);

    // read a byte from ring buffer. Returns true on success, false otherwise
    bool read_byte(uint8_t *data);

    /*
      update bytes at the read pointer. Used to update an object without
      popping it
    */
    bool update(const uint8_t *data, uint32_t len);

    // return size of ringbuffer
    uint32_t get_size(void) const { return size; }

    // set size of ringbuffer, caller responsible for locking
    bool set_size(uint32_t size);

    // advance the read pointer (discarding bytes)
    bool advance(uint32_t n);

    // Returns the pointer and size to a contiguous read of the next available data
    const uint8_t *readptr(uint32_t &available_bytes);

    // peek one byte without advancing read pointer. Return byte
    // or -1 if none available
    int16_t peek(uint32_t ofs) const;

    /*
      read len bytes without advancing the read pointer
    */
    uint32_t peekbytes(uint8_t *data, uint32_t len);

    // Similar to peekbytes(), but will fill out IoVec struct with
    // both parts of the ring buffer if wraparound is happening, or
    // just one part. Returns the number of parts written to.
    struct IoVec {
        uint8_t *data;
        uint32_t len;
    };
    uint8_t peekiovec(IoVec vec[2], uint32_t len);

    // Reserve `len` bytes and fills out `vec` with both parts of the
    // ring buffer (if wraparound is happening), or just one contiguous
    // part. Returns the number of `vec` elements filled out. Can be used
    // with system calls such as `readv()`.
    //
    // After a call to 'reserve()', 'write()' should never be called
    // until 'commit()' is called!
    uint8_t reserve(IoVec vec[2], uint32_t len);

    /*
     * "Releases" the memory previously reserved by 'reserve()' to be read.
     * Committer must inform how many bytes were actually written in 'len'.
     */
    bool commit(uint32_t len);

private:
    uint8_t *buf;
    uint32_t size;

    std::atomic<uint32_t> head{0}; // where to read data
    std::atomic<uint32_t> tail{0}; // where to write data
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

    // Discards the buffer content, emptying it.
    void clear(void)
    {
        buffer->clear();
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

    // push N objects
    bool push(const T *object, uint32_t n) {
        if (buffer->space() < n*sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)object, n*sizeof(T)) == n*sizeof(T);
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
     * push_force() N objects
     */
    bool push_force(const T *object, uint32_t n) {
        uint32_t _space = buffer->space();
        if (_space < sizeof(T)*n) {
            buffer->advance(sizeof(T)*(n-_space));
        }
        return push(object, n);
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
    ObjectArray(uint16_t size_) {
        _size = size_;
        _head = _count = 0;
        _buffer = new T[_size];
    }
    ~ObjectArray(void) {
        delete[] _buffer;
    }

    // return total number of objects
    uint16_t size(void) const {
        return _size;
    }

    // return number of objects available to be read
    uint16_t available(void) const {
        return _count;
    }

    // return number of objects that could be written
    uint16_t space(void) const {
        return _size - _count;
    }

    // true is available() == 0
    bool empty(void) const {
        return _count == 0;
    }

    // push one object
    bool push(const T &object) {
        if (space() == 0) {
            return false;
        }
        _buffer[(_head+_count)%_size] = object;
        _count++;
        return true;
    }

    /*
      throw away an object
     */
    bool pop(void) {
        if (empty()) {
            return false;
        }
        _head = (_head+1) % _size;
        _count--;
        return true;
    }

    // Discards the buffer content, emptying it.
    void clear(void)
    {
        _head = _count = 0;
    }

    /*
      pop earliest object off the queue
     */
    bool pop(T &object) {
        if (empty()) {
            return false;
        }
        object = _buffer[_head];
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
        if (n >= _count) {
            return false;
        }
        if (n == _count-1) {
            // remove last element
            _count--;
            return true;
        }
        if (n == 0) {
            // remove first element
            return pop();
        }
        // take advantage of the [] operator for simple shift of the array elements
        for (uint16_t i=n; i<_count-1; i++) {
            *(*this)[i] = *(*this)[i+1];
        }
        _count--;
        return true;
    }

    // allow array indexing, based on current head. Returns a pointer
    // to the object or nullptr
    T * operator[](uint16_t i) {
        if (i >= _count) {
            return nullptr;
        }
        return &_buffer[(_head+i)%_size];
    }

private:
    T *_buffer;
    uint16_t _size;  // total buffer size
    uint16_t _count; // number in buffer now
    uint16_t _head;  // first element
};
