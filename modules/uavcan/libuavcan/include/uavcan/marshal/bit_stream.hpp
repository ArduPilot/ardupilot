/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_MARSHAL_BIT_STREAM_HPP_INCLUDED
#define UAVCAN_MARSHAL_BIT_STREAM_HPP_INCLUDED

#include <cassert>
#include <uavcan/std.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/transport/abstract_transfer_buffer.hpp>
#include <uavcan/build_config.hpp>

namespace uavcan
{
/**
 * This function implements fast copy of unaligned bit arrays. It isn't part of the library API, so it is not exported.
 * @param src_org       Source array
 * @param src_offset    Bit offset of the first source byte
 * @param src_len       Number of bits to copy
 * @param dst_org       Destination array
 * @param dst_offset    Bit offset of the first destination byte
 */
void bitarrayCopy(const unsigned char* src_org, std::size_t src_offset, std::size_t src_len,
                  unsigned char* dst_org, std::size_t dst_offset);

/**
 * This class treats a chunk of memory as an array of bits.
 * It is used by the bit codec for serialization/deserialization.
 */
class UAVCAN_EXPORT BitStream
{
    static const unsigned MaxBytesPerRW = 16;

    ITransferBuffer& buf_;
    unsigned bit_offset_;
    uint8_t byte_cache_;

    static inline unsigned bitlenToBytelen(unsigned bits) { return (bits + 7) / 8; }

    static inline void copyBitArrayAlignedToUnaligned(const uint8_t* src_org, unsigned src_len,
                                                      uint8_t* dst_org, unsigned dst_offset)
    {
        bitarrayCopy(reinterpret_cast<const unsigned char*>(src_org), 0, src_len,
                     reinterpret_cast<unsigned char*>(dst_org), dst_offset);
    }

    static inline void copyBitArrayUnalignedToAligned(const uint8_t* src_org, unsigned src_offset, unsigned src_len,
                                                      uint8_t* dst_org)
    {
        bitarrayCopy(reinterpret_cast<const unsigned char*>(src_org), src_offset, src_len,
                     reinterpret_cast<unsigned char*>(dst_org), 0);
    }

public:
    static const unsigned MaxBitsPerRW = MaxBytesPerRW * 8;

    enum
    {
        ResultOutOfBuffer = 0,
        ResultOk          = 1
    };

    explicit BitStream(ITransferBuffer& buf)
        : buf_(buf)
        , bit_offset_(0)
        , byte_cache_(0)
    {
        StaticAssert<sizeof(uint8_t) == 1>::check();
    }

    /**
     * Write/read calls interpret bytes as bit arrays, 8 bits per byte, where the most
     * significant bits have lower index, i.e.:
     *   Hex:     55       2d
     *   Bits:    01010101 00101101
     *   Indices: 0  ..  7 8  ..  15
     * Return values:
     *   Negative - Error
     *   Zero     - Out of buffer space
     *   Positive - OK
     */
    int write(const uint8_t* bytes, const unsigned bitlen);
    int read(uint8_t* bytes, const unsigned bitlen);

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

}

#endif // UAVCAN_MARSHAL_BIT_STREAM_HPP_INCLUDED
