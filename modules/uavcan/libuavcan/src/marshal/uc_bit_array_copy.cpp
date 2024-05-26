/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/bit_stream.hpp>
#include <climits>
#include <cstring>
#include <cstddef>

namespace uavcan
{
void bitarrayCopy(const unsigned char* src, std::size_t src_offset, std::size_t src_len,
                  unsigned char* dst, std::size_t dst_offset)
{
    /*
     * Should never be called on a zero-length buffer. The caller will also ensure that the bit
     * offsets never exceed one byte.
     */

    UAVCAN_ASSERT(src_len > 0U);
    UAVCAN_ASSERT(src_offset < 8U && dst_offset < 8U);

    const std::size_t last_bit = src_offset + src_len;
    while (last_bit - src_offset)
    {
        const uint8_t src_bit_offset = src_offset % 8U;
        const uint8_t dst_bit_offset = dst_offset % 8U;

        // The number of bits to copy
        const uint8_t max_offset = uavcan::max(src_bit_offset, dst_bit_offset);
        const std::size_t copy_bits = uavcan::min(last_bit - src_offset, std::size_t(8U - max_offset));

        /*
         * The mask indicating which bits of dest to update:
         * dst_byte_offset           copy_bits          write_mask
         * 0                         8                  11111111
         * 0                         7                  11111110
         * ...
         * 0                         1                  10000000
         * ...
         * 4                         4                  00001111
         * 4                         3                  00001110
         * 4                         2                  00001100
         * 4                         1                  00001000
         * ...
         * 7                         1                  00000001
         */
        const uint8_t write_mask = uint8_t(uint8_t(0xFF00U >> copy_bits) >> dst_bit_offset);

        // The value to be extracted from src, shifted into the dst location
        const uint8_t src_data = uint8_t((src[src_offset / 8U] << src_bit_offset) >> dst_bit_offset);

        dst[dst_offset / 8U] = uint8_t((dst[dst_offset / 8U] & ~write_mask) | (src_data & write_mask));

        src_offset += copy_bits;
        dst_offset += copy_bits;
    }
}
}
