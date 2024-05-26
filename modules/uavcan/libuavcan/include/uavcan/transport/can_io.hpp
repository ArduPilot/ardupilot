/*
 * CAN bus IO logic.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_CAN_IO_HPP_INCLUDED
#define UAVCAN_TRANSPORT_CAN_IO_HPP_INCLUDED

#include <cassert>
#include <uavcan/error.hpp>
#include <uavcan/std.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan/driver/system_clock.hpp>
#include <uavcan/time.hpp>

namespace uavcan
{

struct UAVCAN_EXPORT CanRxFrame : public CanFrame
{
    MonotonicTime ts_mono;
    UtcTime ts_utc;
    uint8_t iface_index;

    CanRxFrame()
        : iface_index(0)
    { }

#if UAVCAN_TOSTRING
    std::string toString(StringRepresentation mode = StrTight) const;
#endif
};


class UAVCAN_EXPORT CanTxQueue : Noncopyable
{
public:
    enum Qos { Volatile, Persistent };

    struct Entry : public LinkedListNode<Entry>  // Not required to be packed - fits the block in any case
    {
        MonotonicTime deadline;
        CanFrame frame;
        uint8_t qos;
        CanIOFlags flags;

        Entry(const CanFrame& arg_frame, MonotonicTime arg_deadline, Qos arg_qos, CanIOFlags arg_flags)
            : deadline(arg_deadline)
            , frame(arg_frame)
            , qos(uint8_t(arg_qos))
            , flags(arg_flags)
        {
            UAVCAN_ASSERT((qos == Volatile) || (qos == Persistent));
            IsDynamicallyAllocatable<Entry>::check();
        }

        static void destroy(Entry*& obj, IPoolAllocator& allocator);

        bool isExpired(MonotonicTime timestamp) const { return timestamp > deadline; }

        bool qosHigherThan(const CanFrame& rhs_frame, Qos rhs_qos) const;
        bool qosLowerThan(const CanFrame& rhs_frame, Qos rhs_qos) const;
        bool qosHigherThan(const Entry& rhs) const { return qosHigherThan(rhs.frame, Qos(rhs.qos)); }
        bool qosLowerThan(const Entry& rhs)  const { return qosLowerThan(rhs.frame, Qos(rhs.qos)); }

#if UAVCAN_TOSTRING
        std::string toString() const;
#endif
    };

private:
    class PriorityInsertionComparator
    {
        const CanFrame& frm_;
    public:
        explicit PriorityInsertionComparator(const CanFrame& frm) : frm_(frm) { }
        bool operator()(const Entry* entry)
        {
            UAVCAN_ASSERT(entry);
            return frm_.priorityHigherThan(entry->frame);
        }
    };

    LinkedListRoot<Entry> queue_;
    LimitedPoolAllocator allocator_;
    ISystemClock& sysclock_;
    uint32_t rejected_frames_cnt_;

    void registerRejectedFrame();

public:
    CanTxQueue(IPoolAllocator& allocator, ISystemClock& sysclock, std::size_t allocator_quota)
        : allocator_(allocator, allocator_quota)
        , sysclock_(sysclock)
        , rejected_frames_cnt_(0)
    { }

    ~CanTxQueue();

    void push(const CanFrame& frame, MonotonicTime tx_deadline, Qos qos, CanIOFlags flags);

    Entry* peek();               // Modifier
    void remove(Entry*& entry);
    const CanFrame* getTopPriorityPendingFrame() const;

    /// The 'or equal' condition is necessary to avoid frame reordering.
    bool topPriorityHigherOrEqual(const CanFrame& rhs_frame) const;

    uint32_t getRejectedFrameCount() const { return rejected_frames_cnt_; }

    bool isEmpty() const { return queue_.isEmpty(); }
};


struct UAVCAN_EXPORT CanIfacePerfCounters
{
    uint64_t frames_tx;
    uint64_t frames_rx;
    uint64_t errors;

    CanIfacePerfCounters()
        : frames_tx(0)
        , frames_rx(0)
        , errors(0)
    { }
};


class UAVCAN_EXPORT CanIOManager : Noncopyable
{
    struct IfaceFrameCounters
    {
        uint64_t frames_tx;
        uint64_t frames_rx;

        IfaceFrameCounters()
            : frames_tx(0)
            , frames_rx(0)
        { }
    };

    ICanDriver& driver_;
    ISystemClock& sysclock_;

    LazyConstructor<CanTxQueue> tx_queues_[MaxCanIfaces];
    IfaceFrameCounters counters_[MaxCanIfaces];

    const uint8_t num_ifaces_;

    int sendToIface(uint8_t iface_index, const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags);
    int sendFromTxQueue(uint8_t iface_index);
    int callSelect(CanSelectMasks& inout_masks, const CanFrame* (& pending_tx)[MaxCanIfaces],
                   MonotonicTime blocking_deadline);

public:
    CanIOManager(ICanDriver& driver, IPoolAllocator& allocator, ISystemClock& sysclock,
                 std::size_t mem_blocks_per_iface = 0);

    uint8_t getNumIfaces() const { return num_ifaces_; }

    CanIfacePerfCounters getIfacePerfCounters(uint8_t iface_index) const;

    const ICanDriver& getCanDriver() const { return driver_; }
    ICanDriver& getCanDriver()             { return driver_; }

    uint8_t makePendingTxMask() const;

    /**
     * Returns:
     *  0 - rejected/timedout/enqueued
     *  1+ - sent/received
     *  negative - failure
     */
    int send(const CanFrame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline,
             uint8_t iface_mask, CanTxQueue::Qos qos, CanIOFlags flags);
    int receive(CanRxFrame& out_frame, MonotonicTime blocking_deadline, CanIOFlags& out_flags);
};

}

#endif // UAVCAN_TRANSPORT_CAN_IO_HPP_INCLUDED
