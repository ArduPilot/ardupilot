/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_NODE_STATUS_MONITOR_HPP_INCLUDED
#define UAVCAN_PROTOCOL_NODE_STATUS_MONITOR_HPP_INCLUDED

#include <uavcan/debug.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{
/**
 * This class implements the core functionality of a network monitor.
 * It can be extended by inheritance to add more complex logic, or used directly as is.
 */
class UAVCAN_EXPORT NodeStatusMonitor
{
public:
    struct NodeStatus
    {
        uint8_t health   : 2;
        uint8_t mode     : 3;
        uint8_t sub_mode : 3;

        NodeStatus() :
            health(protocol::NodeStatus::HEALTH_CRITICAL),
            mode(protocol::NodeStatus::MODE_OFFLINE),
            sub_mode(0)
        {
            StaticAssert<protocol::NodeStatus::FieldTypes::health::BitLen   == 2>::check();
            StaticAssert<protocol::NodeStatus::FieldTypes::mode::BitLen     == 3>::check();
            StaticAssert<protocol::NodeStatus::FieldTypes::sub_mode::BitLen == 3>::check();
        }

        bool operator!=(const NodeStatus rhs) const { return !operator==(rhs); }
        bool operator==(const NodeStatus rhs) const
        {
            return std::memcmp(this, &rhs, sizeof(NodeStatus)) == 0;
        }

#if UAVCAN_TOSTRING
        std::string toString() const
        {
            char buf[40];
            (void)snprintf(buf, sizeof(buf), "health=%d mode=%d sub_mode=%d", int(health), int(mode), int(sub_mode));
            return std::string(buf);
        }
#endif
    };

    struct NodeStatusChangeEvent
    {
        NodeID node_id;
        NodeStatus status;
        NodeStatus old_status;
        bool was_known;

        NodeStatusChangeEvent() :
            was_known(false)
        { }
    };

private:
    enum { TimerPeriodMs100 = 2 };

    typedef MethodBinder<NodeStatusMonitor*,
                         void (NodeStatusMonitor::*)(const ReceivedDataStructure<protocol::NodeStatus>&)>
            NodeStatusCallback;

    typedef MethodBinder<NodeStatusMonitor*, void (NodeStatusMonitor::*)(const TimerEvent&)> TimerCallback;

    Subscriber<protocol::NodeStatus, NodeStatusCallback> sub_;

    TimerEventForwarder<TimerCallback> timer_;

    struct Entry
    {
        NodeStatus status;
        int8_t time_since_last_update_ms100;
        Entry() :
            time_since_last_update_ms100(-1)
        { }
    };

    mutable Entry entries_[NodeID::Max];  // [1, NodeID::Max]

    Entry& getEntry(NodeID node_id) const
    {
        if (node_id.get() < 1 || node_id.get() > NodeID::Max)
        {
            handleFatalError("NodeStatusMonitor NodeID");
        }
        return entries_[node_id.get() - 1];
    }

    void changeNodeStatus(const NodeID node_id, const Entry new_entry_value)
    {
        Entry& entry = getEntry(node_id);
        if (entry.status != new_entry_value.status)
        {
            NodeStatusChangeEvent event;
            event.node_id    = node_id;
            event.old_status = entry.status;
            event.status     = new_entry_value.status;
            event.was_known  = entry.time_since_last_update_ms100 >= 0;

            UAVCAN_TRACE("NodeStatusMonitor", "Node %i [%s] status change: [%s] --> [%s]", int(node_id.get()),
                         (event.was_known ? "known" : "new"),
                         event.old_status.toString().c_str(), event.status.toString().c_str());

            handleNodeStatusChange(event);
        }
        entry = new_entry_value;
    }

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        Entry new_entry;
        new_entry.time_since_last_update_ms100 = 0;
        new_entry.status.health   = msg.health   & ((1 << protocol::NodeStatus::FieldTypes::health::BitLen) - 1);
        new_entry.status.mode     = msg.mode     & ((1 << protocol::NodeStatus::FieldTypes::mode::BitLen) - 1);
        new_entry.status.sub_mode = msg.sub_mode & ((1 << protocol::NodeStatus::FieldTypes::sub_mode::BitLen) - 1);

        changeNodeStatus(msg.getSrcNodeID(), new_entry);

        handleNodeStatusMessage(msg);
    }

    void handleTimerEvent(const TimerEvent&)
    {
        const int OfflineTimeoutMs100 = protocol::NodeStatus::OFFLINE_TIMEOUT_MS / 100;

        for (uint8_t i = 1; i <= NodeID::Max; i++)
        {
            Entry& entry = getEntry(i);
            if (entry.time_since_last_update_ms100 >= 0 &&
                entry.status.mode != protocol::NodeStatus::MODE_OFFLINE)
            {
                entry.time_since_last_update_ms100 =
                    int8_t(entry.time_since_last_update_ms100 + int8_t(TimerPeriodMs100));

                if (entry.time_since_last_update_ms100 > OfflineTimeoutMs100)
                {
                    Entry new_entry_value = entry;
                    new_entry_value.time_since_last_update_ms100 = OfflineTimeoutMs100;
                    new_entry_value.status.mode = protocol::NodeStatus::MODE_OFFLINE;
                    changeNodeStatus(i, new_entry_value);
                }
            }
        }
    }

protected:
    /**
     * Called when a node becomes online, changes status or goes offline.
     * Refer to uavcan.protocol.NodeStatus for the offline timeout value.
     * Overriding is not required.
     */
    virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event)
    {
        (void)event;
    }

    /**
     * Called for every received message uavcan.protocol.NodeStatus after handleNodeStatusChange(), even
     * if the status code did not change.
     * Overriding is not required.
     */
    virtual void handleNodeStatusMessage(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        (void)msg;
    }

public:
    explicit NodeStatusMonitor(INode& node)
        : sub_(node)
        , timer_(node)
    { }

    virtual ~NodeStatusMonitor() { }

    /**
     * Starts the monitor.
     * Destroy the object to stop it.
     * Returns negative error code.
     */
    int start()
    {
        const int res = sub_.start(NodeStatusCallback(this, &NodeStatusMonitor::handleNodeStatus));
        if (res >= 0)
        {
            timer_.setCallback(TimerCallback(this, &NodeStatusMonitor::handleTimerEvent));
            timer_.startPeriodic(MonotonicDuration::fromMSec(TimerPeriodMs100 * 100));
        }
        return res;
    }

    /**
     * Make the node unknown.
     */
    void forgetNode(NodeID node_id)
    {
        if (node_id.isValid())
        {
            Entry& entry = getEntry(node_id);
            entry = Entry();
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * Make all nodes unknown.
     */
    void forgetAllNodes()
    {
        for (unsigned i = 0; i < (sizeof(entries_) / sizeof(entries_[0])); i++)
        {
            entries_[i] = Entry();
        }
    }

    /**
     * Returns status of a given node.
     * Unknown nodes are considered offline.
     * Complexity O(1).
     */
    NodeStatus getNodeStatus(NodeID node_id) const
    {
        if (!node_id.isValid())
        {
            UAVCAN_ASSERT(0);
            return NodeStatus();
        }

        const Entry& entry = getEntry(node_id);
        if (entry.time_since_last_update_ms100 >= 0)
        {
            return entry.status;
        }
        else
        {
            return NodeStatus();
        }
    }

    /**
     * Whether the class has observed this node at least once since initialization.
     * Complexity O(1).
     */
    bool isNodeKnown(NodeID node_id) const
    {
        if (!node_id.isValid())
        {
            UAVCAN_ASSERT(0);
            return false;
        }
        return getEntry(node_id).time_since_last_update_ms100 >= 0;
    }

    /**
     * This helper method allows to quickly estimate the overall network health.
     * Health of the local node is not considered.
     * Returns an invalid Node ID value if there's no known nodes in the network.
     */
    NodeID findNodeWithWorstHealth() const
    {
        NodeID nid_with_worst_health;
        uint8_t worst_health = protocol::NodeStatus::HEALTH_OK;

        for (uint8_t i = 1; i <= NodeID::Max; i++)
        {
            const NodeID nid(i);
            UAVCAN_ASSERT(nid.isUnicast());
            const Entry& entry = getEntry(nid);
            if (entry.time_since_last_update_ms100 >= 0)
            {
                if (entry.status.health > worst_health || !nid_with_worst_health.isValid())
                {
                    nid_with_worst_health = nid;
                    worst_health = entry.status.health;
                }
            }
        }

        return nid_with_worst_health;
    }

    /**
     * Calls the operator for every known node.
     * Operator signature:
     *   void (NodeID, NodeStatus)
     */
    template <typename Operator>
    void forEachNode(Operator op) const
    {
        for (uint8_t i = 1; i <= NodeID::Max; i++)
        {
            const NodeID nid(i);
            UAVCAN_ASSERT(nid.isUnicast());
            const Entry& entry = getEntry(nid);
            if (entry.time_since_last_update_ms100 >= 0)
            {
                op(nid, entry.status);
            }
        }
    }
};

}

#endif // UAVCAN_PROTOCOL_NODE_STATUS_MONITOR_HPP_INCLUDED
