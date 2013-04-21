// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Buffer.h
/// @brief	fifo buffer template class

#include <AP_Buffer.h>
#include <stdint.h>

// Constructor
template <class T, uint8_t SIZE>
AP_Buffer<T,SIZE>::AP_Buffer() :
	_num_items(0)
{
	// clear the buffer
	clear();
}

// clear - removes all points from the curve
template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::clear() {
	// clear the curve
	_num_items = 0;
    _head = 0;
}

// add - adds an item to the buffer.  returns TRUE if successfully added
template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::add( T item )
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
T AP_Buffer<T,SIZE>::peek(uint8_t position) const
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

template float AP_Buffer<float,5>::peek(uint8_t position) const;
template AP_Buffer<float, 5>::AP_Buffer();
template void AP_Buffer<float, 5>::clear();
template void AP_Buffer<float, 5>::add(float);

template float AP_Buffer<float,15>::peek(uint8_t position) const;
template AP_Buffer<float, 15>::AP_Buffer();
template void AP_Buffer<float, 15>::clear();
template void AP_Buffer<float, 15>::add(float);
