// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Buffer.h
/// @brief	fifo buffer template class

#ifndef __AP_BUFFER_H__
#define __AP_BUFFER_H__

#include <stdint.h>

/// @class      AP_Buffer
template <class T, uint8_t SIZE>
class AP_Buffer {
public:
    // Constructor
    AP_Buffer();

    // clear - removes all points from the curve
    void clear();

    // add - adds an item to the buffer.  returns TRUE if successfully added
    void add( T item );

    // get - returns the next value in the buffer
    T get();

    // peek - check what the next value in the buffer is but don't pull it off
    T peek(uint8_t position = 0) const;

    // num_values - returns number of values in the buffer
    uint8_t num_items() const { return _num_items; }

private:
    uint8_t     _num_items;             // number of items in the buffer
    uint8_t     _head;                  // first item in the buffer (will be returned with the next get call)
    T           _buff[SIZE];            // x values of each point on the curve
};

// Typedef for convenience - add more as needed
typedef AP_Buffer<float,5> AP_BufferFloat_Size5;
typedef AP_Buffer<float,15> AP_BufferFloat_Size15;

#endif  // __AP_BUFFER_H__
