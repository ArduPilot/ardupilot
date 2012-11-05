// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_Buffer.h
/// @brief	fifo buffer template class

#ifndef AP_BUFFER
#define AP_BUFFER

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library

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
    uint8_t     _tail;                  // last item added to the buffer (most recently added with the add method)
    T           _buff[SIZE];            // x values of each point on the curve
};

// Typedef for convenience
typedef AP_Buffer<uint8_t,8> AP_BufferUInt8_Size8;
typedef AP_Buffer<uint8_t,16> AP_BufferUnt8_Size16;
typedef AP_Buffer<uint8_t,32> AP_BufferUInt8_Size32;
typedef AP_Buffer<uint8_t,64> AP_BufferUInt8_Size64;
typedef AP_Buffer<uint8_t,128> AP_BufferUInt8_Size128;

typedef AP_Buffer<float,10> AP_BufferFloat_Size10;
typedef AP_Buffer<float,15> AP_BufferFloat_Size15;

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
    _tail = 0;
}

// add - adds an item to the buffer.  returns TRUE if successfully added
template <class T, uint8_t SIZE>
bool AP_Buffer<T,SIZE>::add( T item )
{
    // add item at the tail
    _buff[_tail] = item;

    // move tail forward one position
    _tail++;

    // wrap around to front of buffer if required
    if( _tail >= SIZE )
        _tail = 0;

    // if the tail bumps into the head, move the head forward (throw away the oldest item)
    if( _tail == _head ) {
        _head++;
        if( _head >= SIZE )
            _head = 0;
    }else{
        // increment number of items
        _num_items++;
    }

    // indicate success
    return true;

/*   Old method that fails when buffer fills up
	if( _num_items < SIZE ) {
        // add item at the tail
        _buff[_tail] = item;

        // move tail forward one position
		_tail++;

        // wrap around to front of buffer if required
        if( _tail >= SIZE )
            _tail = 0;

        // increment number of items
        _num_items++;

        // indicate success
        return true;
    }else{
        // we do not have room for the new item
        return false;
    }
*/
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
    if( position >= _num_items )
        return 0;

    // wrap around if necessary
    if( j >= SIZE )
        j -= SIZE;

    // return desired value
    return _buff[j];
}

// displays the contents of the curve (for debugging)
/*template <class T, uint8_t SIZE>
void AP_Buffer<T,SIZE>::print_buffer()
{
    uint8_t i;
	Serial.print_P(PSTR("AP_Buffer: "));
	for( i = 0; i<_num_items; i++ ){
        if( i > 0 ) {
            Serial.print_P(PSTR(", "));
        }
		Serial.print(peek(i));
	}
    Serial.println();
}*/

/*
// displays the contents of the curve (for debugging)
template <class Vector3f, uint8_t SIZE>
void AP_Buffer<Vector3f,SIZE>::print_buffer()
{
    // we cannot print Vector3f types
}
*/

#endif  // AP_BUFFER