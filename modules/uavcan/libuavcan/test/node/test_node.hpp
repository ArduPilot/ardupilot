/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#if __GNUC__
# pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <uavcan/node/abstract_node.hpp>
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <memory>
#include <set>
#include <queue>
#include "../transport/can/can.hpp"
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>

struct TestNode : public uavcan::INode
{
    /*
     * This class used to use the simple pool allocator instead:
     *  uavcan::PoolAllocator<uavcan::MemPoolBlockSize * 1024, uavcan::MemPoolBlockSize> pool;
     * It has been replaced because unlike the simple allocator, heap-based one is not tested as extensively.
     * Moreover, heap based allocator prints and error message upon destruction if some memory has not been freed.
     */
    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize> pool;
    uavcan::Scheduler scheduler;
    uint64_t internal_failure_count;

    TestNode(uavcan::ICanDriver& can_driver, uavcan::ISystemClock& clock_driver, uavcan::NodeID self_node_id) :
        pool(1024),
        scheduler(can_driver, pool, clock_driver),
        internal_failure_count(0)
    {
        setNodeID(self_node_id);
    }

    virtual void registerInternalFailure(const char* msg)
    {
        std::cout << "TestNode internal failure: " << msg << std::endl;
        internal_failure_count++;
    }

    virtual uavcan::IPoolAllocator& getAllocator() { return pool; }
    virtual uavcan::Scheduler& getScheduler() { return scheduler; }
    virtual const uavcan::Scheduler& getScheduler() const { return scheduler; }
};


struct PairableCanDriver : public uavcan::ICanDriver, public uavcan::ICanIface
{
    uavcan::ISystemClock& clock;
    std::set<PairableCanDriver*> others;
    std::queue<uavcan::CanFrame> read_queue;
    std::queue<uavcan::CanFrame> loopback_queue;
    uint64_t error_count;

    PairableCanDriver(uavcan::ISystemClock& clock)
        : clock(clock)
        , error_count(0)
    { }

    void linkTogether(PairableCanDriver* with)
    {
        this->others.insert(with);
        with->others.insert(this);
        others.erase(this);
    }

    virtual uavcan::ICanIface* getIface(uavcan::uint8_t iface_index)
    {
        return (iface_index == 0) ? this : UAVCAN_NULLPTR;
    }
    virtual const uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) const
    {
        return (iface_index == 0) ? this : UAVCAN_NULLPTR;
    }

    virtual uavcan::uint8_t getNumIfaces() const { return 1; }

    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
                                   const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                                   uavcan::MonotonicTime blocking_deadline)
    {
        if (inout_masks.read == 1)
        {
            inout_masks.read = (!read_queue.empty() || !loopback_queue.empty()) ? 1 : 0;
        }
        if (inout_masks.read || inout_masks.write)
        {
            return 1;
        }
        while (clock.getMonotonic() < blocking_deadline)
        {
            usleep(1000);
        }
        return 0;
    }

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime, uavcan::CanIOFlags flags)
    {
        assert(!others.empty());
        for (std::set<PairableCanDriver*>::iterator it = others.begin(); it != others.end(); ++it)
        {
            (*it)->read_queue.push(frame);
        }
        if (flags & uavcan::CanIOFlagLoopback)
        {
            loopback_queue.push(frame);
        }
        return 1;
    }

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
    {
        out_flags = 0;
        if (loopback_queue.empty())
        {
            assert(read_queue.size());
            out_frame = read_queue.front();
            read_queue.pop();
        }
        else
        {
            out_flags |= uavcan::CanIOFlagLoopback;
            out_frame = loopback_queue.front();
            loopback_queue.pop();
        }
        out_ts_monotonic = clock.getMonotonic();
        out_ts_utc = clock.getUtc();
        return 1;
    }

    void pushRxToAllIfaces(const uavcan::CanFrame& can_frame)
    {
        read_queue.push(can_frame);
    }

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig*, uavcan::uint16_t) { return -1; }
    virtual uavcan::uint16_t getNumFilters() const { return 0; }
    virtual uavcan::uint64_t getErrorCount() const { return error_count; }
};


template <typename ClockType>
struct InterlinkedTestNodes
{
    ClockType clock_a;
    ClockType clock_b;
    PairableCanDriver can_a;
    PairableCanDriver can_b;
    TestNode a;
    TestNode b;

    InterlinkedTestNodes(uavcan::NodeID nid_first, uavcan::NodeID nid_second)
        : can_a(clock_a)
        , can_b(clock_b)
        , a(can_a, clock_a, nid_first)
        , b(can_b, clock_b, nid_second)
    {
        can_a.linkTogether(&can_b);
    }

    InterlinkedTestNodes()
        : can_a(clock_a)
        , can_b(clock_b)
        , a(can_a, clock_a, 1)
        , b(can_b, clock_b, 2)
    {
        can_a.linkTogether(&can_b);
    }

    int spinBoth(uavcan::MonotonicDuration duration)
    {
        assert(!duration.isNegative());
        unsigned nspins2 = unsigned(duration.toMSec() / 2);
        nspins2 = nspins2 ? nspins2 : 1;
        while (nspins2 --> 0)
        {
            int ret = a.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (ret < 0)
            {
                return ret;
            }
            ret = b.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (ret < 0)
            {
                return ret;
            }
        }
        return 0;
    }
};


typedef InterlinkedTestNodes<SystemClockDriver> InterlinkedTestNodesWithSysClock;
typedef InterlinkedTestNodes<SystemClockMock> InterlinkedTestNodesWithClockMock;


template <unsigned NumNodes>
struct TestNetwork
{
    struct NodeEnvironment
    {
        SystemClockDriver clock;
        PairableCanDriver can_driver;
        TestNode node;

        NodeEnvironment(uavcan::NodeID node_id)
            : can_driver(clock)
            , node(can_driver, clock, node_id)
        { }
    };

    std::unique_ptr<NodeEnvironment> nodes[NumNodes];

    TestNetwork(uavcan::uint8_t first_node_id = 1)
    {
        for (uavcan::uint8_t i = 0; i < NumNodes; i++)
        {
            nodes[i].reset(new NodeEnvironment(uint8_t(first_node_id + i)));
        }

        for (uavcan::uint8_t i = 0; i < NumNodes; i++)
        {
            for (uavcan::uint8_t k = 0; k < NumNodes; k++)
            {
                nodes[i]->can_driver.linkTogether(&nodes[k]->can_driver);
            }
        }

        for (uavcan::uint8_t i = 0; i < NumNodes; i++)
        {
            assert(nodes[i]->can_driver.others.size() == (NumNodes - 1));
        }
    }

    int spinAll(uavcan::MonotonicDuration duration)
    {
        assert(!duration.isNegative());
        unsigned nspins = unsigned(duration.toMSec() / NumNodes);
        nspins = nspins ? nspins : 1;
        while (nspins --> 0)
        {
            for (uavcan::uint8_t i = 0; i < NumNodes; i++)
            {
                int ret = nodes[i]->node.spin(uavcan::MonotonicDuration::fromMSec(1));
                if (ret < 0)
                {
                    return ret;
                }
            }
        }
        return 0;
    }

    TestNode& operator[](unsigned index)
    {
        if (index >= NumNodes)
        {
            throw std::out_of_range("No such test node");
        }
        return nodes[index]->node;
    }
};
