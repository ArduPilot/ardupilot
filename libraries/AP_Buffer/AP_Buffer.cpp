// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Buffer.h
/// @brief	fifo buffer template class

#include <AP_Buffer.h>
#include <stdint.h>

// Constructor
template <class T, uint8_t SIZE>
AP_Buffer<T,SIZE>::AP_Buffer() :
	_num_items(0), _head(0)
{
}

// clear - removes all points from the curve
template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::clear() {
	// clear the curve
	_num_items = 0;
    _head = 0;
}

template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::push_back( const T &item )
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

template <class T, uint8_t SIZE>
T AP_Buffer<T,SIZE>::pop_front()
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

template <class T, uint8_t SIZE>
const T& AP_Buffer<T,SIZE>::peek(uint8_t position) const
{
    uint8_t j = _head + position;

    // return zero if position is out of range
    if( position >= _num_items ) {
    	const static T r = 0;
        return r;
    }

    // wrap around if necessary
    if( j >= SIZE )
        j -= SIZE;

    // return desired value
    return _buff[j];
}

// explicitly instantiate all needed template instances
// (this is needed to allow the separation of declaration and definition into header and source files)
template class AP_Buffer<float,  5>;
template class AP_Buffer<float, 15>;
