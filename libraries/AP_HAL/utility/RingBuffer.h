#pragma once

#include <atomic>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>

/*
 * Circular buffer of bytes.
 */
class ByteBuffer {
public:
    ByteBuffer(uint32_t size);
    ByteBuffer(uint8_t* _buf, uint32_t _size) :
    buf(_buf),
    size(_size),
    external_buf(true)
    {}
    ~ByteBuffer(void);

    // number of bytes available to be read
    uint32_t available(void) const;

    // Discards the buffer content, emptying it.
    void clear(void);

    // number of bytes space available to write
    uint32_t space(void) const;

    // true if available() is zero
    bool is_empty(void) const WARN_IF_UNUSED;

    // write bytes to ringbuffer. Returns number of bytes written
    uint32_t write(const uint8_t *data, uint32_t len);

    // read bytes from ringbuffer. Returns number of bytes read
    uint32_t read(uint8_t *data, uint32_t len);

    // read a byte from ring buffer. Returns true on success, false otherwise
    bool read_byte(uint8_t *data) WARN_IF_UNUSED;

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

    bool external_buf;
};

/*
  ring buffer class for objects of fixed size
  !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
 */
template <class T>
class ObjectBuffer {
public:
    ObjectBuffer(uint32_t _size = 0) {
        // we set size to 1 more than requested as the byte buffer
        // gives one less byte than requested. We round up to a full
        // multiple of the object size so that we always get aligned
        // elements, which makes the readptr() method possible
        buffer = new ByteBuffer(((_size+1) * sizeof(T)));
        external_buf = false;
    }

    ObjectBuffer(ByteBuffer *_buffer) :
    buffer(_buffer),
    external_buf(true)
    {}

    ~ObjectBuffer(void) {
        if (!external_buf)
            delete buffer;
    }

    // return size of ringbuffer
    uint32_t get_size(void) const {
        if (buffer == nullptr) {
            return 0;
        }
        uint32_t size = buffer->get_size() / sizeof(T);
        return size>0?size-1:0;
    }

    // set size of ringbuffer, caller responsible for locking
    bool set_size(uint32_t size) { return buffer->set_size(((size+1) * sizeof(T))); }

    // read len objects without advancing the read pointer
    uint32_t peek(T *data, uint32_t len) { return buffer->peekbytes((uint8_t*)data, len * sizeof(T)) / sizeof(T); }

    // Discards the buffer content, emptying it.
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    void clear(void)
    {
        buffer->clear();
    }

    // return number of objects available to be read from the front of the queue
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    uint32_t available(void) const {
        return buffer->available() / sizeof(T);
    }

    // return number of objects that could be written to the back of the queue
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    uint32_t space(void) const {
        return buffer->space() / sizeof(T);
    }

    // true is available() == 0
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool is_empty(void) const WARN_IF_UNUSED {
        return buffer->is_empty();
    }

    // push one object onto the back of the queue
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool push(const T &object) {
        if (buffer->space() < sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    // push N objects onto the back of the queue
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool push(const T *object, uint32_t n) {
        if (buffer->space() < n*sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)object, n*sizeof(T)) == n*sizeof(T);
    }
    
    /*
      throw away an object from the front of the queue
     */
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool pop(void) {
        return buffer->advance(sizeof(T));
    }

    /*
      pop earliest object off the front of the queue
     */
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool pop(T &object) WARN_IF_UNUSED {
        if (buffer->available() < sizeof(T)) {
            return false;
        }
        return buffer->read((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }


    /*
     * push_force() is semantically equivalent to:
     *   if (!push(t)) { pop(); push(t); }
     */
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool push_force(const T &object) {
        if (buffer->space() < sizeof(T)) {
            buffer->advance(sizeof(T));
        }
        return push(object);
    }

    /*
     * push_force() N objects
     */
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool push_force(const T *object, uint32_t n) {
        uint32_t _space = buffer->space();
        if (_space < sizeof(T)*n) {
            buffer->advance(sizeof(T)*(n-_space));
        }
        return push(object, n);
    }
    
    /*
      peek copies an object out from the front of the queue without advancing the read pointer
     */
    // !!! Note ObjectBuffer_TS is a duplicate of this update, in both places !!!
    bool peek(T &object) WARN_IF_UNUSED {
        return buffer->peekbytes((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    /*
      return a pointer to first contiguous array of available
      objects. Return nullptr if none available
     */
    // !!! Note ObjectBuffer_TS is a duplicate of this, update in both places !!!
    const T *readptr(uint32_t &n) {
        uint32_t avail_bytes = 0;
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wcast-align"
        const T *ret = (const T *)buffer->readptr(avail_bytes);
        #pragma GCC diagnostic pop
        if (!ret || avail_bytes < sizeof(T)) {
            return nullptr;
        }
        n = avail_bytes / sizeof(T);
        return ret;
    }

    // advance the read pointer (discarding objects)
    // !!! Note ObjectBuffer_TS is a duplicate of this, update in both places !!!
    bool advance(uint32_t n) {
        return buffer->advance(n * sizeof(T));
    }
    
    /* update the object at the front of the queue (the one that would
       be fetched by pop()) */
    // !!! Note ObjectBuffer_TS is a duplicate of this, update in both places !!!
    bool update(const T &object) {
        return buffer->update((uint8_t*)&object, sizeof(T));
    }

private:
    ByteBuffer *buffer = nullptr;
    bool external_buf = true;
};

/*
  Thread safe ring buffer class for objects of fixed size
  !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
 */
template <class T>
class ObjectBuffer_TS {
public:
    ObjectBuffer_TS(uint32_t _size = 0) {
        // we set size to 1 more than requested as the byte buffer
        // gives one less byte than requested. We round up to a full
        // multiple of the object size so that we always get aligned
        // elements, which makes the readptr() method possible
        buffer = new ByteBuffer(((_size+1) * sizeof(T)));
    }
    ~ObjectBuffer_TS(void) {
        delete buffer;
    }

    // return size of ringbuffer
    uint32_t get_size(void) {
        WITH_SEMAPHORE(sem);
        if (buffer == nullptr) {
            return 0;
        }
        uint32_t size = buffer->get_size() / sizeof(T);
        return size>0?size-1:0;
    }

    // set size of ringbuffer, caller responsible for locking
    bool set_size(uint32_t size) {
        WITH_SEMAPHORE(sem);
        return buffer->set_size(((size+1) * sizeof(T)));
    }

    // read len objects without advancing the read pointer
    uint32_t peek(T *data, uint32_t len) {
        WITH_SEMAPHORE(sem);
        return buffer->peekbytes((uint8_t*)data, len * sizeof(T)) / sizeof(T);
    }


    // Discards the buffer content, emptying it.
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    void clear(void)
    {
        WITH_SEMAPHORE(sem);
        buffer->clear();
    }

    // return number of objects available to be read from the front of the queue
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    uint32_t available(void) {
        WITH_SEMAPHORE(sem);
        return buffer->available() / sizeof(T);
    }

    // return number of objects that could be written to the back of the queue
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    uint32_t space(void) {
        WITH_SEMAPHORE(sem);
        return buffer->space() / sizeof(T);
    }

    // true is available() == 0
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool is_empty(void) WARN_IF_UNUSED {
        WITH_SEMAPHORE(sem);
        return buffer->is_empty();
    }

    // push one object onto the back of the queue
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool push(const T &object) {
        WITH_SEMAPHORE(sem);
        if (buffer->space() < sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    // push N objects onto the back of the queue
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool push(const T *object, uint32_t n) {
        WITH_SEMAPHORE(sem);
        if (buffer->space() < n*sizeof(T)) {
            return false;
        }
        return buffer->write((uint8_t*)object, n*sizeof(T)) == n*sizeof(T);
    }

    /*
      throw away an object from the front of the queue
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool pop(void) {
        WITH_SEMAPHORE(sem);
        return buffer->advance(sizeof(T));
    }

    /*
      pop earliest object off the front of the queue
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool pop(T &object) WARN_IF_UNUSED {
        WITH_SEMAPHORE(sem);
        if (buffer->available() < sizeof(T)) {
            return false;
        }
        return buffer->read((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    /*
     * push_force() is semantically equivalent to:
     *   if (!push(t)) { pop(); push(t); }
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool push_force(const T &object) {
        WITH_SEMAPHORE(sem);
        if (buffer->space() < sizeof(T)) {
            buffer->advance(sizeof(T));
        }
        return push(object);
    }

    /*
     * push_force() N objects
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool push_force(const T *object, uint32_t n) {
        WITH_SEMAPHORE(sem);
        uint32_t _space = buffer->space();
        if (_space < sizeof(T)*n) {
            buffer->advance(sizeof(T)*(n-_space));
        }
        return push(object, n);
    }

    /*
      peek copies an object out from the front of the queue without advancing the read pointer
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool peek(T &object) WARN_IF_UNUSED {
        WITH_SEMAPHORE(sem);
        return buffer->peekbytes((uint8_t*)&object, sizeof(T)) == sizeof(T);
    }

    /*
      return a pointer to first contiguous array of available
      objects. Return nullptr if none available
     */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    const T *readptr(uint32_t &n) {
        WITH_SEMAPHORE(sem);
        uint32_t avail_bytes = 0;
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wcast-align"
        const T *ret = (const T *)buffer->readptr(avail_bytes);
        #pragma GCC diagnostic pop
        if (!ret || avail_bytes < sizeof(T)) {
            return nullptr;
        }
        n = avail_bytes / sizeof(T);
        return ret;
    }

    // advance the read pointer (discarding objects)
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool advance(uint32_t n) {
        WITH_SEMAPHORE(sem);
        return buffer->advance(n * sizeof(T));
    }

    /* update the object at the front of the queue (the one that would
       be fetched by pop()) */
    // !!! Note this is a duplicate of ObjectBuffer with semaphore, update in both places !!!
    bool update(const T &object) {
        WITH_SEMAPHORE(sem);
        return buffer->update((uint8_t*)&object, sizeof(T));
    }

private:
    ByteBuffer *buffer = nullptr;
    HAL_Semaphore sem;
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
    bool is_empty(void) const WARN_IF_UNUSED {
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
    bool pop(void) WARN_IF_UNUSED {
        if (is_empty()) {
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
    bool pop(T &object) WARN_IF_UNUSED {
        if (is_empty()) {
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
            UNUSED_RESULT(pop());
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

typedef ObjectBuffer<float> FloatBuffer;
typedef ObjectBuffer_TS<float> FloatBuffer_TS;
typedef ObjectArray<float> FloatArray;
