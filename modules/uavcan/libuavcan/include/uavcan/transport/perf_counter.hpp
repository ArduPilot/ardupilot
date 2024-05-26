/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_PERF_COUNTER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_PERF_COUNTER_HPP_INCLUDED

#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>

namespace uavcan
{

#if UAVCAN_TINY

class UAVCAN_EXPORT TransferPerfCounter : Noncopyable
{
public:
    void addTxTransfer() { }
    void addRxTransfer() { }
    void addError() { }
    void addErrors(unsigned) { }
    uint64_t getTxTransferCount() const { return 0; }
    uint64_t getRxTransferCount() const { return 0; }
    uint64_t getErrorCount() const { return 0; }
};

#else

/**
 * The class is declared noncopyable for two reasons:
 *  - to prevent accidental pass-by-value into a mutator
 *  - to make the addresses of the counters fixed and exposable to the user of the library
 */
class UAVCAN_EXPORT TransferPerfCounter : Noncopyable
{
    uint64_t transfers_tx_;
    uint64_t transfers_rx_;
    uint64_t errors_;

public:
    TransferPerfCounter()
        : transfers_tx_(0)
        , transfers_rx_(0)
        , errors_(0)
    { }

    void addTxTransfer() { transfers_tx_++; }
    void addRxTransfer() { transfers_rx_++; }

    void addError() { errors_++; }

    void addErrors(unsigned errors)
    {
        errors_ += errors;
    }

    /**
     * Returned references are guaranteed to be valid as long as this instance of Node exists.
     * This is enforced by virtue of the class being Noncopyable.
     */
    const uint64_t& getTxTransferCount() const { return transfers_tx_; }
    const uint64_t& getRxTransferCount() const { return transfers_rx_; }
    const uint64_t& getErrorCount() const { return errors_; }
};

#endif

}

#endif // UAVCAN_TRANSPORT_PERF_COUNTER_HPP_INCLUDED
