// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Buffer.h
/// @brief	fifo (queue) buffer template class

#ifndef __AP_BUFFER_H__
#define __AP_BUFFER_H__

#include <stdint.h>

/// @class      AP_Buffer
template <class T, uint8_t SIZE>
class AP_Buffer {
public:
    /**
     * constructs an empty queue.
     */
    AP_Buffer();

    /**
     * removes all elements from the queue.
     */
    void clear();

    /**
     * adds an item to the end of the buffer.
     *
     * If the buffer is full, the oldest element (i.e. the element at the begin) is removed.
     *
     * @param item
     */
    void push_back( const T &item );

    /**
     * removes an element from the begin of the buffer (i.e. the oldest element) and returns it.
     *
     * If the buffer is empty, 0 is returned.
     *
     * @return
     */
    T pop_front();

    /**
     * Returns a reference to an element of the buffer.
     *
     * If position isn't valid (i.e. >= size(); ) 0 is returned.
     *
     * @param position : index of the element
     * 						- "0"          is the oldest ( front(); )
     * 						- "size() - 1" is the newest ( back();  )
     * @return
     */
    const T& peek(uint8_t position) const;

    /**
     * Return a reference to the element at the begin of the queue (i.e. the oldest element).
     *
     * If the queue is empty, 0 is returned.
     *
     * @return : oldest element
     */
    const T& front() const { return this->peek(0); }

    /**
     * Returns a reference to the element at the end of the queue (i.e. the newest element).
     *
     * If the queue is empty, 0 is returned.
     *
     * @return : newest element
     */
    const T& back() const  { return this->peek( this->size() - 1); }


    /**
     * Returns the number of elements in the queue.
     * @return
     */
    uint8_t size() const { return _num_items; }

    /**
     * Return whether the queue is full (i.e. size() == SIZE).
     * @return
     */
    bool is_full() const { return _num_items >= SIZE; }

    /**
     * Return whether the queue is empty.
     * @return
     */
    bool is_empty() const { return _num_items == 0; }

private:
    uint8_t     _num_items;             // number of items in the buffer
    uint8_t     _head;                  // first item in the buffer (will be returned with the next pop_front call)
    T           _buff[SIZE];            // x values of each point on the curve
};

// Typedef for convenience - add more as needed
// (The used template instances have also to be instantiated explicitly in the source file.)
typedef AP_Buffer<float,5> AP_BufferFloat_Size5;
typedef AP_Buffer<float,15> AP_BufferFloat_Size15;

#endif  // __AP_BUFFER_H__
