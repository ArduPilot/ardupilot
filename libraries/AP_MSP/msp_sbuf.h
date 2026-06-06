/*
  Code ported and adapted from betaflight/src/main/common/streambuf.h
 */
#pragma once
#include "msp.h"
#include <stdint.h>

#if HAL_MSP_ENABLED

namespace MSP
{
typedef struct sbuf_s {
    uint8_t *ptr;          // data pointer must be first (sbuff_t* is equivalent to uint8_t **)
    uint8_t *end;
} sbuf_t;

//helper functions
uint8_t* sbuf_ptr(sbuf_t *buf);
uint16_t sbuf_bytes_remaining(const sbuf_t *buf);
bool sbuf_check_bounds(const sbuf_t *buf, const uint8_t len);
void sbuf_switch_to_reader(sbuf_t *buf, uint8_t *base);
void sbuf_write_data(sbuf_t *dst, const void *data, int len);
// little-endian reads; advance the read pointer, returning 0 if the buffer is exhausted
uint8_t sbuf_read_u8(sbuf_t *src);
uint16_t sbuf_read_u16(sbuf_t *src);
}

#endif //HAL_MSP_ENABLED