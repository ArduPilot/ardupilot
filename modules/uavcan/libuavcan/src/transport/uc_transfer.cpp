/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/can_io.hpp>

namespace uavcan
{
/**
 * TransferPriority
 */
const uint8_t TransferPriority::BitLen;
const uint8_t TransferPriority::NumericallyMax;
const uint8_t TransferPriority::NumericallyMin;

const TransferPriority TransferPriority::Default((1U << BitLen) / 2);
const TransferPriority TransferPriority::MiddleLower((1U << BitLen) / 2 + (1U << BitLen) / 4);
const TransferPriority TransferPriority::OneHigherThanLowest(NumericallyMax - 1);
const TransferPriority TransferPriority::OneLowerThanHighest(NumericallyMin + 1);
const TransferPriority TransferPriority::Lowest(NumericallyMax);

/**
 * TransferID
 */
const uint8_t TransferID::BitLen;
const uint8_t TransferID::Max;
const uint8_t TransferID::Half;

/**
 * NodeID
 */
const uint8_t NodeID::ValueBroadcast;
const uint8_t NodeID::ValueInvalid;
const uint8_t NodeID::BitLen;
const uint8_t NodeID::Max;
const uint8_t NodeID::MaxRecommendedForRegularNodes;
const NodeID NodeID::Broadcast(ValueBroadcast);

/**
 * TransferID
 */
int TransferID::computeForwardDistance(TransferID rhs) const
{
    int d = int(rhs.get()) - int(get());
    if (d < 0)
    {
        d += 1 << BitLen;
    }

    UAVCAN_ASSERT(((get() + d) & Max) == rhs.get());
    return d;
}

}
