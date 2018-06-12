/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file ring_buffer.h
 * @brief Simple circular buffer
 *
 * This implementation is not thread-safe.  In particular, none of
 * these functions is guaranteed re-entrant.
 */

#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include "hal_types.h"

/**
 * Ring buffer type.
 *
 * The buffer is empty when head == tail.
 *
 * The buffer is full when the head is one byte in front of the tail,
 * modulo buffer length.
 *
 * One byte is left free to distinguish empty from full. */
typedef struct ring_buffer {
    volatile uint8_t *buf; /**< Buffer items are stored into */
    uint16_t head;         /**< Index of the next item to remove */
    uint16_t tail;         /**< Index where the next item will get inserted */
    uint16_t size;         /**< Buffer capacity minus one */
} ring_buffer;


#ifdef __cplusplus
  extern "C" {
#endif

/**
 * Initialise a ring buffer.
 *
 *  @param rb   Instance to initialise
 *
 *  @param size Number of items in buf.  The ring buffer will always
 *              leave one element unoccupied, so the maximum number of
 *              elements it can store will be size - 1.  Thus, size
 *              must be at least 2.
 *
 *  @param buf  Buffer to store items into
 */
static inline void rb_init(ring_buffer *rb, uint16_t size, uint8_t *buf) {
    rb->head = 0;
    rb->tail = 0;
    rb->size = size - 1;
    rb->buf = buf;
}

/**
 * @brief Return the number of elements stored in the ring buffer.
 * @param rb Buffer whose elements to count.
 */
static inline uint16_t rb_full_count(ring_buffer *rb) {
    uint16_t t=rb->tail;
    uint16_t h=rb->head;
    
    int32_t size = t - h;
    if (t < h) {
        size += rb->size + 1;
    }
    return (uint16_t)size;
}

/**
 * @brief Returns true if and only if the ring buffer is full.
 * @param rb Buffer to test.
 */
static inline int rb_is_full(ring_buffer *rb) {
    uint16_t t=rb->tail;
    uint16_t h=rb->head;
    return (t + 1 == h || (t == rb->size && h == 0) );
}

/**
 * @brief Returns true if and only if the ring buffer is empty.
 * @param rb Buffer to test.
 */
static inline int rb_is_empty(ring_buffer *rb) {
    return rb->head == rb->tail;
}

/**
 * Append element onto the end of a ring buffer.
 * @param rb Buffer to append onto.
 * @param element Value to append.
 */
static inline void rb_insert(ring_buffer *rb, uint8_t element) {
    uint16_t t=rb->tail;
    rb->buf[t] = element;
    rb->tail = (t == rb->size) ? 0 : t + 1;
}

/**
 * @brief Remove and return the first item from a ring buffer.
 * @param rb Buffer to remove from, must contain at least one element.
 */
static inline uint8_t rb_remove(ring_buffer *rb) {
    uint16_t h=rb->head;
    uint8_t ch = rb->buf[h];
    rb->head = (h == rb->size) ? 0 : h + 1;
    return ch;
}

/**
 * @brief Attempt to remove the first item from a ring buffer.
 *
 * If the ring buffer is nonempty, removes and returns its first item.
 * If it is empty, does nothing and returns a negative value.
 *
 * @param rb Buffer to attempt to remove from.
 */
static inline int16_t rb_safe_remove(ring_buffer *rb) {
    return rb_is_empty(rb) ? -1 : rb_remove(rb);
}

/**
 * @brief Attempt to insert an element into a ring buffer.
 *
 * @param rb Buffer to insert into.
 * @param element Value to insert into rb.
 * @sideeffect If rb is not full, appends element onto buffer.
 * @return If element was appended, then true; otherwise, false. */
static inline int rb_safe_insert(ring_buffer *rb, uint8_t element) {
    if (rb_is_full(rb)) {
        return 0;
    }
    rb_insert(rb, element);
    return 1;
}

/**
 * @brief Append an item onto the end of a non-full ring buffer.
 *
 * If the buffer is full, removes its first item, then inserts the new
 * element at the end.
 *
 * @param rb Ring buffer to insert into.
 * @param element Value to insert into ring buffer.
 * @return On success, returns -1.  If an element was popped, returns
 *         the popped value.
 */
static inline int rb_push_insert(ring_buffer *rb, uint8_t element) {
    int ret = -1;
    if (rb_is_full(rb)) {
        ret = rb_remove(rb);
    }
    rb_insert(rb, element);
    return ret;
}

/**
 * @brief Discard all items from a ring buffer.
 * @param rb Ring buffer to discard all items from.
 */
static inline void rb_reset(ring_buffer *rb) {
    rb->tail = rb->head;
}

#ifdef __cplusplus
  }
#endif

#endif

