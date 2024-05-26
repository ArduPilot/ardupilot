/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_UTIL_BITSET_HPP_INCLUDED
#define UAVCAN_UTIL_BITSET_HPP_INCLUDED

#include <cassert>
#include <cstddef>
#include <cstring>
#include <uavcan/build_config.hpp>

namespace uavcan
{
/**
 * STL-like bitset
 */
template <std::size_t NumBits>
class UAVCAN_EXPORT BitSet
{
    enum { NumBytes = (NumBits + 7) / 8 };

    static std::size_t getByteNum(std::size_t bit_num) { return bit_num / 8; }

    static std::size_t getBitNum(const std::size_t bit_num) { return bit_num % 8; }

    static void validatePos(std::size_t& inout_pos)
    {
        if (inout_pos >= NumBits)
        {
            UAVCAN_ASSERT(0);
            inout_pos = NumBits - 1;
        }
    }

    char data_[NumBytes];

public:
    class Reference
    {
        friend class BitSet;

        BitSet* const parent_;
        const std::size_t bitpos_;

        Reference(BitSet* arg_parent, std::size_t arg_bitpos)
            : parent_(arg_parent)
            , bitpos_(arg_bitpos)
        { }

    public:
        Reference& operator=(bool x)
        {
            parent_->set(bitpos_, x);
            return *this;
        }

        Reference& operator=(const Reference& x)
        {
            parent_->set(bitpos_, x);
            return *this;
        }

        bool operator~() const
        {
            return !parent_->test(bitpos_);
        }

        operator bool() const
        {
            return parent_->test(bitpos_);
        }
    };

    BitSet()
        : data_()
    {
        reset();
    }

    BitSet<NumBits>& reset()
    {
        std::memset(data_, 0, NumBytes);
        return *this;
    }

    BitSet<NumBits>& set()
    {
        std::memset(data_, 0xFF, NumBytes);
        return *this;
    }

    BitSet<NumBits>& set(std::size_t pos, bool val = true)
    {
        validatePos(pos);
        if (val)
        {
            data_[getByteNum(pos)] = char(data_[getByteNum(pos)] | (1 << getBitNum(pos)));
        }
        else
        {
            data_[getByteNum(pos)] = char(data_[getByteNum(pos)] & ~(1 << getBitNum(pos)));
        }
        return *this;
    }

    bool test(std::size_t pos) const
    {
        return (data_[getByteNum(pos)] & (1 << getBitNum(pos))) != 0;
    }

    bool any() const
    {
        for (std::size_t i = 0; i < NumBits; ++i)
        {
            if (test(i))
            {
                return true;
            }
        }
        return false;
    }

    std::size_t count() const
    {
        std::size_t retval = 0;
        for (std::size_t i = 0; i < NumBits; ++i)
        {
            retval += test(i) ? 1U : 0U;
        }
        return retval;
    }

    std::size_t size() const { return NumBits; }

    bool operator[](std::size_t pos) const
    {
        return test(pos);
    }

    Reference operator[](std::size_t pos)
    {
        validatePos(pos);
        return Reference(this, pos);
    }

    BitSet<NumBits>& operator=(const BitSet<NumBits> & rhs)
    {
        if (&rhs == this)
        {
            return *this;
        }
        for (std::size_t i = 0; i < NumBytes; ++i)
        {
            data_[i] = rhs.data_[i];
        }
        return *this;
    }

    bool operator!=(const BitSet<NumBits>& rhs) const { return !operator==(rhs); }
    bool operator==(const BitSet<NumBits>& rhs) const
    {
        for (std::size_t i = 0; i < NumBits; ++i)
        {
            if (test(i) != rhs.test(i))
            {
                return false;
            }
        }
        return true;
    }
};

template <> class BitSet<0>;  ///< Invalid instantiation


template <typename Stream, std::size_t NumBits>
Stream& operator<<(Stream& s, const BitSet<NumBits>& x)
{
    for (std::size_t i = NumBits; i > 0; --i)
    {
        s << (x.test(i-1) ? "1" : "0");
    }
    return s;
}

}

#endif // UAVCAN_UTIL_BITSET_HPP_INCLUDED
