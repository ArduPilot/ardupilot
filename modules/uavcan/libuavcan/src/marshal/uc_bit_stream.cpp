/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/bit_stream.hpp>
#include <uavcan/transport/transfer_buffer.hpp>
#include <cassert>

namespace uavcan
{

const unsigned BitStream::MaxBytesPerRW;
const unsigned BitStream::MaxBitsPerRW;

int BitStream::write(const uint8_t* bytes, const unsigned bitlen)
{
    // Temporary buffer is needed to merge new bits with cached unaligned bits from the last write() (see byte_cache_)
    uint8_t tmp[MaxBytesPerRW + 1];

    // Tmp space must be large enough to accomodate new bits AND unaligned bits from the last write()
    const unsigned bytelen = bitlenToBytelen(bitlen + (bit_offset_ % 8));
    UAVCAN_ASSERT(MaxBytesPerRW >= bytelen);
    tmp[0] = tmp[bytelen - 1] = 0;

    fill(tmp, tmp + bytelen, uint8_t(0));
    copyBitArrayAlignedToUnaligned(bytes, bitlen, tmp, bit_offset_ % 8);

    const unsigned new_bit_offset = bit_offset_ + bitlen;

    // Bitcopy algorithm resets skipped bits in the first byte. Restore them back.
    tmp[0] |= byte_cache_;

    // (new_bit_offset % 8 == 0) means that this write was perfectly aligned.
    byte_cache_ = uint8_t((new_bit_offset % 8) ? tmp[bytelen - 1] : 0);

    /*
     * Dump the data into the destination buffer.
     * Note that if this write was unaligned, last written byte in the buffer will be rewritten with updated value
     * within the next write() operation.
     */
    const int write_res = buf_.write(bit_offset_ / 8, tmp, bytelen);
    if (write_res < 0)
    {
        return write_res;
    }
    if (static_cast<unsigned>(write_res) < bytelen)
    {
        return ResultOutOfBuffer;
    }

    bit_offset_ = new_bit_offset;
    return ResultOk;
}

int BitStream::read(uint8_t* bytes, const unsigned bitlen)
{
    uint8_t tmp[MaxBytesPerRW + 1];

    const unsigned bytelen = bitlenToBytelen(bitlen + (bit_offset_ % 8));
    UAVCAN_ASSERT(MaxBytesPerRW >= bytelen);

    const int read_res = buf_.read(bit_offset_ / 8, tmp, bytelen);
    if (read_res < 0)
    {
        return read_res;
    }
    if (static_cast<unsigned>(read_res) < bytelen)
    {
        return ResultOutOfBuffer;
    }

    fill(bytes, bytes + bitlenToBytelen(bitlen), uint8_t(0));
    copyBitArrayUnalignedToAligned(tmp, bit_offset_ % 8, bitlen, bytes);
    bit_offset_ += bitlen;
    return ResultOk;
}

#if UAVCAN_TOSTRING
std::string BitStream::toString() const
{
    std::string out;
    out.reserve(128);

    for (unsigned offset = 0; true; offset++)
    {
        uint8_t byte = 0;
        if (1 != buf_.read(offset, &byte, 1U))
        {
            break;
        }
        for (int i = 7; i >= 0; i--)     // Most significant goes first
        {
            out += (byte & (1 << i)) ? '1' : '0';
        }
        out += ' ';
    }
    if (out.length() > 0)
    {
        (void)out.erase(out.length() - 1, 1);
    }
    return out;
}
#endif

}
