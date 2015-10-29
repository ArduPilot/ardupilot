#ifndef __AP_HAL_UTILITY_RINGBUFFER_H__
#define __AP_HAL_UTILITY_RINGBUFFER_H__

/*
  common ring buffer handling macros

  These macros assume a ring buffer like this:

    uint8_t *_buf;
    uint16_t _buf_size;
    volatile uint16_t _buf_head;
    volatile uint16_t _buf_tail;

  These should be converted to a class in future
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size

#endif // __AP_HAL_UTILITY_RINGBUFFER_H__
