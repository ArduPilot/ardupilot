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
     * If the buffer is empty, the result is undefined.
     *
     * Use is_empty() to check if your call will be valid.
     *
     * @return
     */
    T pop_front();

    /**
     * Check if an index for accessing an element is valid.
     * @param index
     * @return
     */
    bool index_is_valid(uint8_t index) const { return index < this->size(); }

    /**
     * Returns a reference to an element of the buffer.
     *
     * If index isn't valid (i.e. >= size(); ) the behavior is undefined.
     *
     * Use index_is_valid(uin8_t) to verify your argument.
     *
     * @param index : index of the element
     * 						- "0"          is the oldest ( front(); )
     * 						- "size() - 1" is the newest ( back();  )
     * @return
     */
    const T& peek(uint8_t index) const;

    /**
     * Return a reference to the element at the begin of the queue (i.e. the oldest element).
     *
     * If the queue is empty, the behavior is undefined.
     *
     * Use is_empty() to check if your call will be valid.
     *
     * @return : oldest element
     */
    const T& front() const { return _buff[_head]; }

    /**
     * Returns a reference to the element at the end of the queue (i.e. the newest element).
     *
     * If the queue is empty, the behavior is undefined.
     *
     * Use is_empty() to check if your call will be valid.
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
     * Return whether the queue is full (i.e. size() >= SIZE).
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
typedef AP_Buffer<float,5> AP_BufferFloat_Size5;
typedef AP_Buffer<float,15> AP_BufferFloat_Size15;

template <class T, uint8_t SIZE>
AP_Buffer<T,SIZE>::AP_Buffer() :
	_num_items(0), _head(0)
{
}

template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::clear() {
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
	// store the result
    T result = this->front();

    // remove value from queue
    // -----------------------
    // pop on an empty queue yields an undefined result.
    // We perform the empty-check, to keep the queue in a defined state (i.e. _head and _num_items remain valid)
    // and prevent memory corruption.
    if( not this->is_empty() ) {
		// increment to next point
		_head++;
		if( _head >= SIZE )
			_head = 0;

		// reduce number of items
		_num_items--;
    }
    return result;
}

template <class T, uint8_t SIZE>
const T& AP_Buffer<T,SIZE>::peek(uint8_t position) const
{
    uint8_t j = _head + position;

    // wrap around if necessary
    if( j >= SIZE )
        j -= SIZE;

    // return desired value
    return _buff[j];
}

#endif  // __AP_BUFFER_H__
