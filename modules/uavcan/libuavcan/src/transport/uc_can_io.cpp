/*
 * CAN bus IO logic.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/can_io.hpp>
#include <uavcan/debug.hpp>
#include <cassert>

namespace uavcan
{
/*
 * CanRxFrame
 */
#if UAVCAN_TOSTRING
std::string CanRxFrame::toString(StringRepresentation mode) const
{
    std::string out = CanFrame::toString(mode);
    out.reserve(128);
    out += " ts_m="   + ts_mono.toString();
    out += " ts_utc=" + ts_utc.toString();
    out += " iface=";
    out += char('0' + iface_index);
    return out;
}
#endif

/*
 * CanTxQueue::Entry
 */
void CanTxQueue::Entry::destroy(Entry*& obj, IPoolAllocator& allocator)
{
    if (obj != UAVCAN_NULLPTR)
    {
        obj->~Entry();
        allocator.deallocate(obj);
        obj = UAVCAN_NULLPTR;
    }
}

bool CanTxQueue::Entry::qosHigherThan(const CanFrame& rhs_frame, Qos rhs_qos) const
{
    if (qos != rhs_qos)
    {
        return qos > rhs_qos;
    }
    return frame.priorityHigherThan(rhs_frame);
}

bool CanTxQueue::Entry::qosLowerThan(const CanFrame& rhs_frame, Qos rhs_qos) const
{
    if (qos != rhs_qos)
    {
        return qos < rhs_qos;
    }
    return frame.priorityLowerThan(rhs_frame);
}

#if UAVCAN_TOSTRING
std::string CanTxQueue::Entry::toString() const
{
    std::string str_qos;
    switch (qos)
    {
    case Volatile:
    {
        str_qos = "<volat> ";
        break;
    }
    case Persistent:
    {
        str_qos = "<perst> ";
        break;
    }
    default:
    {
        UAVCAN_ASSERT(0);
        str_qos = "<?WTF?> ";
        break;
    }
    }
    return str_qos + frame.toString();
}
#endif

/*
 * CanTxQueue
 */
CanTxQueue::~CanTxQueue()
{
    Entry* p = queue_.get();
    while (p)
    {
        Entry* const next = p->getNextListNode();
        remove(p);
        p = next;
    }
}

void CanTxQueue::registerRejectedFrame()
{
    if (rejected_frames_cnt_ < NumericTraits<uint32_t>::max())
    {
        rejected_frames_cnt_++;
    }
}

void CanTxQueue::push(const CanFrame& frame, MonotonicTime tx_deadline, Qos qos, CanIOFlags flags)
{
    const MonotonicTime timestamp = sysclock_.getMonotonic();

    if (timestamp >= tx_deadline)
    {
        UAVCAN_TRACE("CanTxQueue", "Push rejected: already expired");
        registerRejectedFrame();
        return;
    }

    void* praw = allocator_.allocate(sizeof(Entry));
    if (praw == UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("CanTxQueue", "Push OOM #1, cleanup");
        // No memory left in the pool, so we try to remove expired frames
        Entry* p = queue_.get();
        while (p)
        {
            Entry* const next = p->getNextListNode();
            if (p->isExpired(timestamp))
            {
                UAVCAN_TRACE("CanTxQueue", "Push: Expired %s", p->toString().c_str());
                registerRejectedFrame();
                remove(p);
            }
            p = next;
        }
        praw = allocator_.allocate(sizeof(Entry));         // Try again
    }

    if (praw == UAVCAN_NULLPTR)
    {
        UAVCAN_TRACE("CanTxQueue", "Push OOM #2, QoS arbitration");
        registerRejectedFrame();

        // Find a frame with lowest QoS
        Entry* p = queue_.get();
        if (p == UAVCAN_NULLPTR)
        {
            UAVCAN_TRACE("CanTxQueue", "Push rejected: Nothing to replace");
            return;
        }
        Entry* lowestqos = p;
        while (p)
        {
            if (lowestqos->qosHigherThan(*p))
            {
                lowestqos = p;
            }
            p = p->getNextListNode();
        }
        // Note that frame with *equal* QoS will be replaced too.
        if (lowestqos->qosHigherThan(frame, qos))           // Frame that we want to transmit has lowest QoS
        {
            UAVCAN_TRACE("CanTxQueue", "Push rejected: low QoS");
            return;                                         // What a loser.
        }
        UAVCAN_TRACE("CanTxQueue", "Push: Replacing %s", lowestqos->toString().c_str());
        remove(lowestqos);
        praw = allocator_.allocate(sizeof(Entry));        // Try again
    }

    if (praw == UAVCAN_NULLPTR)
    {
        return;                                            // Seems that there is no memory at all.
    }
    Entry* entry = new (praw) Entry(frame, tx_deadline, qos, flags);
    UAVCAN_ASSERT(entry);
    queue_.insertBefore(entry, PriorityInsertionComparator(frame));
}

CanTxQueue::Entry* CanTxQueue::peek()
{
    const MonotonicTime timestamp = sysclock_.getMonotonic();
    Entry* p = queue_.get();
    while (p)
    {
        if (p->isExpired(timestamp))
        {
            UAVCAN_TRACE("CanTxQueue", "Peek: Expired %s", p->toString().c_str());
            Entry* const next = p->getNextListNode();
            registerRejectedFrame();
            remove(p);
            p = next;
        }
        else
        {
            return p;
        }
    }
    return UAVCAN_NULLPTR;
}

void CanTxQueue::remove(Entry*& entry)
{
    if (entry == UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(0);
        return;
    }
    queue_.remove(entry);
    Entry::destroy(entry, allocator_);
}

const CanFrame* CanTxQueue::getTopPriorityPendingFrame() const
{
    return (queue_.get() == UAVCAN_NULLPTR) ? UAVCAN_NULLPTR : &queue_.get()->frame;
}

bool CanTxQueue::topPriorityHigherOrEqual(const CanFrame& rhs_frame) const
{
    const Entry* entry = queue_.get();
    if (entry == UAVCAN_NULLPTR)
    {
        return false;
    }
    return !rhs_frame.priorityHigherThan(entry->frame);
}

/*
 * CanIOManager
 */
int CanIOManager::sendToIface(uint8_t iface_index, const CanFrame& frame, MonotonicTime tx_deadline, CanIOFlags flags)
{
    UAVCAN_ASSERT(iface_index < MaxCanIfaces);
    ICanIface* const iface = driver_.getIface(iface_index);
    if (iface == UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(0);   // Nonexistent interface
        return -ErrLogic;
    }
    const int res = iface->send(frame, tx_deadline, flags);
    if (res != 1)
    {
        UAVCAN_TRACE("CanIOManager", "Send failed: code %i, iface %i, frame %s",
                     res, iface_index, frame.toString().c_str());
    }
    if (res > 0)
    {
        counters_[iface_index].frames_tx += unsigned(res);
    }
    return res;
}

int CanIOManager::sendFromTxQueue(uint8_t iface_index)
{
    UAVCAN_ASSERT(iface_index < MaxCanIfaces);
    CanTxQueue::Entry* entry = tx_queues_[iface_index]->peek();
    if (entry == UAVCAN_NULLPTR)
    {
        return 0;
    }
    const int res = sendToIface(iface_index, entry->frame, entry->deadline, entry->flags);
    if (res > 0)
    {
        tx_queues_[iface_index]->remove(entry);
    }
    return res;
}

int CanIOManager::callSelect(CanSelectMasks& inout_masks, const CanFrame* (& pending_tx)[MaxCanIfaces],
                             MonotonicTime blocking_deadline)
{
    const CanSelectMasks in_masks = inout_masks;

    const int res = driver_.select(inout_masks, pending_tx, blocking_deadline);
    if (res < 0)
    {
        return -ErrDriver;
    }

    inout_masks.read  &= in_masks.read;  // Driver is not required to clean the masks
    inout_masks.write &= in_masks.write;
    return res;
}

CanIOManager::CanIOManager(ICanDriver& driver, IPoolAllocator& allocator, ISystemClock& sysclock,
                           std::size_t mem_blocks_per_iface)
    : driver_(driver)
    , sysclock_(sysclock)
    , num_ifaces_(driver.getNumIfaces())
{
    if (num_ifaces_ < 1 || num_ifaces_ > MaxCanIfaces)
    {
        handleFatalError("Num ifaces");
    }

    if (mem_blocks_per_iface == 0)
    {
        mem_blocks_per_iface = allocator.getBlockCapacity() / (num_ifaces_ + 1U) + 1U;
    }
    UAVCAN_TRACE("CanIOManager", "Memory blocks per iface: %u, total: %u",
                 unsigned(mem_blocks_per_iface), unsigned(allocator.getBlockCapacity()));

    for (int i = 0; i < num_ifaces_; i++)
    {
        tx_queues_[i].construct<IPoolAllocator&, ISystemClock&, std::size_t>
        (allocator, sysclock, mem_blocks_per_iface);
    }
}

uint8_t CanIOManager::makePendingTxMask() const
{
    uint8_t write_mask = 0;
    for (uint8_t i = 0; i < getNumIfaces(); i++)
    {
        if (!tx_queues_[i]->isEmpty())
        {
            write_mask = uint8_t(write_mask | (1 << i));
        }
    }
    return write_mask;
}

CanIfacePerfCounters CanIOManager::getIfacePerfCounters(uint8_t iface_index) const
{
    ICanIface* const iface = driver_.getIface(iface_index);
    if (iface == UAVCAN_NULLPTR || iface_index >= MaxCanIfaces)
    {
        UAVCAN_ASSERT(0);
        return CanIfacePerfCounters();
    }
    CanIfacePerfCounters cnt;
    cnt.errors = iface->getErrorCount() + tx_queues_[iface_index]->getRejectedFrameCount();
    cnt.frames_rx = counters_[iface_index].frames_rx;
    cnt.frames_tx = counters_[iface_index].frames_tx;
    return cnt;
}

int CanIOManager::send(const CanFrame& frame, MonotonicTime tx_deadline, MonotonicTime blocking_deadline,
                       uint8_t iface_mask, CanTxQueue::Qos qos, CanIOFlags flags)
{
    const uint8_t num_ifaces = getNumIfaces();
    const uint8_t all_ifaces_mask = uint8_t((1U << num_ifaces) - 1);
    iface_mask &= all_ifaces_mask;

    if (blocking_deadline > tx_deadline)
    {
        blocking_deadline = tx_deadline;
    }

    int retval = 0;

    while (true)        // Somebody please refactor this.
    {
        if (iface_mask == 0)
        {
            break;
        }

        CanSelectMasks masks;
        masks.write = iface_mask | makePendingTxMask();
        {
            // Building the list of next pending frames per iface.
            // The driver will give them a scrutinizing look before deciding whether he wants to accept them.
            const CanFrame* pending_tx[MaxCanIfaces] = {};
            for (int i = 0; i < num_ifaces; i++)
            {
                CanTxQueue& q = *tx_queues_[i];
                if (iface_mask & (1 << i))      // I hate myself so much right now.
                {
                    pending_tx[i] = q.topPriorityHigherOrEqual(frame) ? q.getTopPriorityPendingFrame() : &frame;
                }
                else
                {
                    pending_tx[i] = q.getTopPriorityPendingFrame();
                }
            }

            const int select_res = callSelect(masks, pending_tx, blocking_deadline);
            if (select_res < 0)
            {
                return -ErrDriver;
            }
            UAVCAN_ASSERT(masks.read == 0);
        }

        // Transmission
        for (uint8_t i = 0; i < num_ifaces; i++)
        {
            if (masks.write & (1 << i))
            {
                int res = 0;
                if (iface_mask & (1 << i))
                {
                    if (tx_queues_[i]->topPriorityHigherOrEqual(frame))
                    {
                        res = sendFromTxQueue(i);                 // May return 0 if nothing to transmit (e.g. expired)
                    }
                    if (res <= 0)
                    {
                        res = sendToIface(i, frame, tx_deadline, flags);
                        if (res > 0)
                        {
                            iface_mask &= uint8_t(~(1 << i));     // Mark transmitted
                        }
                    }
                }
                else
                {
                    res = sendFromTxQueue(i);
                }
                if (res > 0)
                {
                    retval++;
                }
            }
        }

        // Timeout. Enqueue the frame if wasn't transmitted and leave.
        const bool timed_out = sysclock_.getMonotonic() >= blocking_deadline;
        if (masks.write == 0 || timed_out)
        {
            if (!timed_out)
            {
                UAVCAN_TRACE("CanIOManager", "Send: Premature timeout in select(), will try again");
                continue;
            }
            for (uint8_t i = 0; i < num_ifaces; i++)
            {
                if (iface_mask & (1 << i))
                {
                    tx_queues_[i]->push(frame, tx_deadline, qos, flags);
                }
            }
            break;
        }
    }
    return retval;
}

int CanIOManager::receive(CanRxFrame& out_frame, MonotonicTime blocking_deadline, CanIOFlags& out_flags)
{
    const uint8_t num_ifaces = getNumIfaces();

    while (true)
    {
        CanSelectMasks masks;
        masks.write = makePendingTxMask();
        masks.read = uint8_t((1 << num_ifaces) - 1);
        {
            const CanFrame* pending_tx[MaxCanIfaces] = {};
            for (int i = 0; i < num_ifaces; i++)      // Dear compiler, kindly unroll this. Thanks.
            {
                pending_tx[i] = tx_queues_[i]->getTopPriorityPendingFrame();
            }

            const int select_res = callSelect(masks, pending_tx, blocking_deadline);
            if (select_res < 0)
            {
                return -ErrDriver;
            }
        }

        // Write - if buffers are not empty, one frame will be sent for each iface per one receive() call
        for (uint8_t i = 0; i < num_ifaces; i++)
        {
            if (masks.write & (1 << i))
            {
                (void)sendFromTxQueue(i);  // It may fail, we don't care. Requested operation was receive, not send.
            }
        }

        // Read
        for (uint8_t i = 0; i < num_ifaces; i++)
        {
            if (masks.read & (1 << i))
            {
                ICanIface* const iface = driver_.getIface(i);
                if (iface == UAVCAN_NULLPTR)
                {
                    UAVCAN_ASSERT(0);   // Nonexistent interface
                    continue;
                }

                const int res = iface->receive(out_frame, out_frame.ts_mono, out_frame.ts_utc, out_flags);
                if (res == 0)
                {
                    UAVCAN_ASSERT(0);   // select() reported that iface has pending RX frames, but receive() returned none
                    continue;
                }
                out_frame.iface_index = i;

                if ((res > 0) && !(out_flags & CanIOFlagLoopback))
                {
                    counters_[i].frames_rx += 1;
                }
                return (res < 0) ? -ErrDriver : res;
            }
        }

        // Timeout checked in the last order - this way we can operate with expired deadline:
        if (sysclock_.getMonotonic() >= blocking_deadline)
        {
            break;
        }
    }
    return 0;
}

}
