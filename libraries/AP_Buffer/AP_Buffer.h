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
    
    /// Constructor
    ///
    AP_Buffer();

    /// clear - removes all elements from the queue
    ///
    void clear();

    /// push_back - adds an item to the end of the buffer.
    /// If the buffer is full, the oldest element (i.e. the element at the begin) is removed
    /// @param item
    void push_back( const T &item );

    /// pop_front - removes an element from the begin of the buffer (i.e. the oldest element)
    /// and returns it.  If the buffer is empty, 0 is returned
    /// @return
    T pop_front();

    /// peek - returns a reference to an element of the buffer
    /// if position isn't valid (i.e. >= size()) 0 is returned
    /// @param position : index of the element
    /// "0" is the oldest, size()-1 is the newest
    /// @return
    const T& peek(uint8_t position) const;

    /// front - return a reference to the element at the begin of the queue (i.e. the oldest element)
    /// If the queue is empty, 0 is returned.
    /// @return : oldest element
    const T& front() const { return this->peek(0); }

    /// size - returns the number of elements in the queue
    /// @return
    uint8_t size() const { return _num_items; }

    /// is_full - return true if the queue is full (i.e. size() == SIZE)
    /// @return
    bool is_full() const { return _num_items >= SIZE; }

    /// is_empty - returns true if the queue is empty
    /// @return
    bool is_empty() const { return _num_items == 0; }

private:
    uint8_t     _num_items;             // number of items in the buffer
    uint8_t     _head;                  // first item in the buffer (will be returned with the next pop_front call)
    T           _buff[SIZE];            // x values of each point on the curve
};

// Typedef for convenience - add more as needed
typedef AP_Buffer<float,5> AP_BufferFloat_Size5;
typedef AP_Buffer<float,15> AP_BufferFloat_Size15;

template <class T, uint8_t SIZE>
AP_Buffer<T,SIZE>::AP_Buffer() :
	_num_items(0), _head(0)
{
}

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

#endif  // __AP_BUFFER_H__
