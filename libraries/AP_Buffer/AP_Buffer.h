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
    virtual void clear();

    // add - adds an item to the buffer.  returns TRUE if successfully added
    virtual bool add( T item );

    // get - returns the next value in the buffer
    virtual T get();

    // peek - check what the next value in the buffer is but don't pull it off
    virtual T peek(uint8_t position = 0);

    // num_values - returns number of values in the buffer
    virtual uint8_t num_items() { return _num_items; }

    // print_buffer - display the contents of the buffer on the serial port
    //virtual void print_buffer();

protected:
    uint8_t     _num_items;             // number of items in the buffer
    uint8_t     _head;                  // first item in the buffer (will be returned with the next get call)
    T           _buff[SIZE];            // x values of each point on the curve
};

// Typedef for convenience
typedef AP_Buffer<uint8_t,8> AP_BufferUInt8_Size8;
typedef AP_Buffer<uint8_t,16> AP_BufferUnt8_Size16;
typedef AP_Buffer<uint8_t,32> AP_BufferUInt8_Size32;
typedef AP_Buffer<uint8_t,64> AP_BufferUInt8_Size64;
typedef AP_Buffer<uint8_t,128> AP_BufferUInt8_Size128;

typedef AP_Buffer<float,5> AP_BufferFloat_Size5;
typedef AP_Buffer<float,10> AP_BufferFloat_Size10;
typedef AP_Buffer<float,15> AP_BufferFloat_Size15;
typedef AP_Buffer<float,20> AP_BufferFloat_Size20;

// Constructor
template <class T, uint8_t SIZE>
AP_Buffer<T,SIZE>::AP_Buffer() :
	_num_items(0)
{
	// clear the buffer
	clear();
};

// clear - removes all points from the curve
template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::clear() {
	// clear the curve
	_num_items = 0;
    _head = 0;
}

// add - adds an item to the buffer.  returns TRUE if successfully added
template <class T, uint8_t SIZE>
bool AP_Buffer<T,SIZE>::add( T item )
{
    // determine position of new item
    uint8_t tail = _head + _num_items;
    if( tail >= SIZE ) {
        tail -= SIZE;
    }

    // add item to buffer
    _buff[tail] = item;

    // increment number of items
    if( _num_items < SIZE ) {
        _num_items++;
    }else{
        // no room for new items so drop oldest item
        _head++;
        if( _head >= SIZE ) {
            _head = 0;
        }
    }

    // indicate success
    return true;
}

// get - returns the next value in the buffer
template <class T, uint8_t SIZE>
T AP_Buffer<T,SIZE>::get()
{
	T result;

	// return zero if buffer is empty
	if( _num_items == 0 ) {
		return 0;
	}

	// get next value in buffer
    result = _buff[_head];

    // increment to next point
    _head++;
    if( _head >= SIZE )
        _head = 0;

    // reduce number of items
    _num_items--;

    // return item
    return result;
}

// peek - check what the next value in the buffer is but don't pull it off
template <class T, uint8_t SIZE>
T AP_Buffer<T,SIZE>::peek(uint8_t position)
{
    uint8_t j = _head+position;

    // return zero if position is out of range
    if( position >= _num_items ) {
        return 0;
    }

    // wrap around if necessary
    if( j >= SIZE )
        j -= SIZE;

    // return desired value
    return _buff[j];
}

#endif  // __AP_BUFFER_H__
