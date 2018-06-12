
/**

(c) 2017 night_ghost@ykoctpa.ru
 
    based on: ring_buffer.h

 * @file ring_buffer_pulse.h
 * @brief Simple circular buffer for PEM input
 *
 * This implementation is not thread-safe.  In particular, none of
 * these functions is guaranteed re-entrant.
 */

#ifndef _PULSE_BUFFER_H_
#define _PULSE_BUFFER_H_

#include "hal_types.h"
#include "pwm_in.h"

#ifdef __cplusplus
  extern "C" {
#endif

/**
 * Ring buffer type.
 *
 * The buffer is empty when head == tail.
 *
 * The buffer is full when the head is one byte in front of the tail,
 * modulo buffer length.
 *
 * One element is left free to distinguish empty from full. */
typedef struct Pulse_buffer {
    Pulse *buf;               /**< Buffer items are stored into */
    uint16_t head;            /**< Index of the next item to remove */
    volatile uint16_t tail;   /**< Index where the next item will get inserted */
    uint16_t size;            /**< Buffer capacity minus one */
} pulse_buffer;



/**
 * Initialise a ring buffer.
 *
 *  @param pb   Instance to initialise
 *
 *  @param size Number of items in buf.  The ring buffer will always
 *              leave one element unoccupied, so the maximum number of
 *              elements it can store will be size - 1.  Thus, size
 *              must be at least 2.
 *
 *  @param buf  Buffer to store items into
 */
static inline void pb_init(volatile pulse_buffer *pb, uint16_t size, Pulse *buf) {
    pb->head = 0;
    pb->tail = 0;
    pb->size = size - 1;
    pb->buf = buf;
}

/**
 * @brief Return the number of elements stored in the ring buffer.
 * @param pb Buffer whose elements to count.
 */
static inline uint16_t pb_full_count(volatile pulse_buffer *pb) {
    uint16_t h=pb->head;
    uint16_t t=pb->tail;
    int32_t size = t - h;
    if (t < h) {
        size += pb->size + 1;
    }
    return (uint16_t)size;
}

/**
 * @brief Returns true if and only if the ring buffer is full.
 * @param pb Buffer to test.
 */
static inline int pb_is_full(volatile pulse_buffer *pb) {
    uint16_t t = pb->tail;
    uint16_t h = pb->head;
    return (t + 1 == h) || (t == pb->size && h == 0);
}

/**
 * @brief Returns true if and only if the ring buffer is empty.
 * @param pb Buffer to test.
 */
static inline int pb_is_empty(volatile pulse_buffer *pb) {
    bool ret=pb->head == pb->tail;
    return ret;
}

/**
 * Append element onto the end of a ring buffer.
 * @param pb Buffer to append onto.
 * @param element Value to append.
 */
static inline void pb_insert(volatile pulse_buffer *pb, Pulse element) { 
    uint16_t t = pb->tail;
    pb->buf[t] = element;
    pb->tail = (t == pb->size) ? 0 : t + 1;
}

/**
 * @brief Remove and return the first item from a ring buffer.
 * @param pb Buffer to remove from, must contain at least one element.
 */
static inline Pulse pb_remove(volatile pulse_buffer *pb) {
    uint16_t h = pb->head;
    Pulse p = pb->buf[h];
    pb->head = (h == pb->size) ? 0 : h + 1;
    return p;
}

/**
 * @brief Attempt to insert an element into a ring buffer.
 *
 * @param pb Buffer to insert into.
 * @param element Value to insert into pb.
 * @sideeffect If pb is not full, appends element onto buffer.
 * @return If element was appended, then 1; otherwise, 0. */
static inline int pb_safe_insert(volatile pulse_buffer *pb, Pulse element) {
    if (pb_is_full(pb)) {
        return 0;
    }
    pb_insert(pb, element);
    return 1;
}


/**
 * @brief Discard all items from a ring buffer.
 * @param pb Ring buffer to discard all items from.
 */
static inline void pb_reset(volatile pulse_buffer *pb) {
    pb->tail = pb->head;
}

#ifdef __cplusplus
  }
#endif

#endif

