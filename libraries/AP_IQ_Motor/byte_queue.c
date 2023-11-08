/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  IQ C++ API is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  IQ C++ API is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
  Name: byte_queue.c
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: James Paulos
  Contributors: Matthew Piccoli, Raphael Van Hoffelen
*/

/// A FIFO buffer for uint8 bytes implemented with a fixed size circular buffer.
/// See header file for usage.

#include "byte_queue.h"


void InitBQ(struct ByteQueue *p, uint8_t* data, uint16_t data_size)
{
    p->data = data;
    p->data_size = data_size;
    p->start = p->data;
    p->end = p->data;
    p->count = 0;
}


int8_t IsFullBQ(struct ByteQueue *p)
{
    // if p->end at end of fixed array
    if (p->end == p->data + p->data_size - 1) {
        if (p->start == p->data) {
            return 1;   // buffer full if p->start is at beginning of fixed array
        } else {
            return 0;   // buffer not full
        }
    }
    // otherwise p->end not at end of fixed array
    else {
        if (p->end == p->start - 1) {
            return 1;   // buffer full if p->end preceeds p->start by one
        } else {
            return 0;   // buffer not full
        }
    }
}


int8_t IsEmptyBQ(struct ByteQueue *p)
{
    if (p->start == p->end) {
        return 1;
    } else {
        return 0;
    }
}


uint16_t CountBQ(struct ByteQueue *p)
{
    return (p->count);
}


uint8_t GetByteBQ(struct ByteQueue *p)
{

    // silent failure if empty
    if (IsEmptyBQ(p)) {
        return (0);
    } else {
        // otherwise buffer contains data
        // get data at start
        uint8_t temp_char = *(p->start);
        // if start is not at the end of the array, advance start
        if (p->start != p->data + p->data_size - 1) {
            p->start = p->start + 1;
        }
        // otherwise wrap start to the beginning of the array
        else {
            p->start = p->data;
        }
        p->count--;
        return temp_char;
    }
}


uint8_t PeekByteBQ(struct ByteQueue *p)
{
    // silent failure if empty
    if (IsEmptyBQ(p)) {
        return (0);
    } else {
        return (*(p->start));
    }
}


int8_t PutByteBQ(struct ByteQueue *p, uint8_t item)
{
    // if p->end points to last element of array
    if (p->end == p->data + p->data_size - 1) {
        // if start is at the beginning of the array, the buffer is full
        if (p->start == p->data) {
            return 0;           // failure, buffer is full
        }
        // otherwise stuff item at p->end
        else {
            *(p->end) = item;   // stuff item at p->end
            p->end = p->data;   // move p->end to start of array
        }
    }
    // otherwise p->end does not point to last element of array
    else {
        // if p->end preceeds p->start by one, buffer is full
        if (p->end == (p->start - 1)) {
            return 0;           // failure, buffer is full
        }
        // otherwise stuff item at p->end
        else {
            *(p->end) = item;       // stuff item at p->end
            p->end = p->end + 1;    // advance p->end one
        }
    }
    p->count++;
    return 1;   // success
}
