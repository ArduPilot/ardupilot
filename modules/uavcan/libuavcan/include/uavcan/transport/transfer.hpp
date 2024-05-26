/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_TRANSFER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_TRANSFER_HPP_INCLUDED

#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/std.hpp>

namespace uavcan
{

static const unsigned GuaranteedPayloadLenPerFrame = 7;            ///< Guaranteed for all transfers, all CAN standards

enum TransferType
{
    TransferTypeServiceResponse  = 0,
    TransferTypeServiceRequest   = 1,
    TransferTypeMessageBroadcast = 2
};

static const uint8_t NumTransferTypes = 3;


class UAVCAN_EXPORT TransferPriority
{
    uint8_t value_;

public:
    static const uint8_t BitLen = 5U;
    static const uint8_t NumericallyMax = (1U << BitLen) - 1;
    static const uint8_t NumericallyMin = 0;

    /// This priority is used by default
    static const TransferPriority Default;
    static const TransferPriority MiddleLower;
    static const TransferPriority OneHigherThanLowest;
    static const TransferPriority OneLowerThanHighest;
    static const TransferPriority Lowest;

    TransferPriority() : value_(0xFF) { }

    TransferPriority(uint8_t value)   // Implicit
        : value_(value)
    {
        UAVCAN_ASSERT(isValid());
    }

    template <uint8_t Percent>
    static TransferPriority fromPercent()
    {
        StaticAssert<(Percent <= 100)>::check();
        enum { Result = ((100U - Percent) * NumericallyMax) / 100U };
        StaticAssert<(Result <= NumericallyMax)>::check();
        StaticAssert<(Result >= NumericallyMin)>::check();
        return TransferPriority(Result);
    }

    uint8_t get() const { return value_; }

    bool isValid() const { return value_ < (1U << BitLen); }

    bool operator!=(TransferPriority rhs) const { return value_ != rhs.value_; }
    bool operator==(TransferPriority rhs) const { return value_ == rhs.value_; }
};


class UAVCAN_EXPORT TransferID
{
    uint8_t value_;

public:
    static const uint8_t BitLen = 5U;
    static const uint8_t Max = (1U << BitLen) - 1U;
    static const uint8_t Half = (1U << BitLen) / 2U;

    TransferID()
        : value_(0)
    { }

    TransferID(uint8_t value)    // implicit
        : value_(value)
    {
        value_ &= Max;
        UAVCAN_ASSERT(value == value_);
    }

    bool operator!=(TransferID rhs) const { return !operator==(rhs); }
    bool operator==(TransferID rhs) const { return get() == rhs.get(); }

    void increment()
    {
        value_ = (value_ + 1) & Max;
    }

    uint8_t get() const
    {
        UAVCAN_ASSERT(value_ <= Max);
        return value_;
    }

    /**
     * Amount of increment() calls to reach rhs value.
     */
    int computeForwardDistance(TransferID rhs) const;
};


class UAVCAN_EXPORT NodeID
{
    static const uint8_t ValueBroadcast = 0;
    static const uint8_t ValueInvalid   = 0xFF;
    uint8_t value_;

public:
    static const uint8_t BitLen = 7U;
    static const uint8_t Max = (1U << BitLen) - 1U;
    static const uint8_t MaxRecommendedForRegularNodes = Max - 2;
    static const NodeID Broadcast;

    NodeID() : value_(ValueInvalid) { }

    NodeID(uint8_t value)   // Implicit
        : value_(value)
    {
        UAVCAN_ASSERT(isValid());
    }

    uint8_t get() const { return value_; }

    bool isValid()     const { return value_ <= Max; }
    bool isBroadcast() const { return value_ == ValueBroadcast; }
    bool isUnicast()   const { return (value_ <= Max) && (value_ != ValueBroadcast); }

    bool operator!=(NodeID rhs) const { return !operator==(rhs); }
    bool operator==(NodeID rhs) const { return value_ == rhs.value_; }

    bool operator<(NodeID rhs) const { return value_ < rhs.value_; }
    bool operator>(NodeID rhs) const { return value_ > rhs.value_; }
    bool operator<=(NodeID rhs) const { return value_ <= rhs.value_; }
    bool operator>=(NodeID rhs) const { return value_ >= rhs.value_; }
};

}

#endif // UAVCAN_TRANSPORT_TRANSFER_HPP_INCLUDED
