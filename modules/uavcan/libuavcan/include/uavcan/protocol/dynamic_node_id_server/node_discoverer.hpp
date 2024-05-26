/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_NODE_DISCOVERER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_NODE_DISCOVERER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/util/map.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/util/bitset.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/protocol/dynamic_node_id_server/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
#include <cassert>
// UAVCAN types
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
/**
 * The allocator must implement this interface.
 */
class INodeDiscoveryHandler
{
public:
    /**
     * In order to avoid bus congestion, normally only leader can discover new nodes.
     */
    virtual bool canDiscoverNewNodes() const = 0;

    /**
     * These values represent the level of awareness of a certain node by the server.
     */
    enum NodeAwareness
    {
        NodeAwarenessUnknown,
        NodeAwarenessKnownButNotCommitted,
        NodeAwarenessKnownAndCommitted
    };

    /**
     * It is OK to do full log traversal in this method, because the unique ID collector will cache the
     * result when possible.
     */
    virtual NodeAwareness checkNodeAwareness(NodeID node_id) const = 0;

    /**
     * This method will be called when a new node responds to GetNodeInfo request.
     * If this method fails to register the node, the node will be queried again later and this method will be
     * invoked again.
     * Unique ID will be UAVCAN_NULLPTR if the node is assumed to not implement the GetNodeInfo service.
     */
    virtual void handleNewNodeDiscovery(const UniqueID* unique_id_or_null, NodeID node_id) = 0;

    virtual ~INodeDiscoveryHandler() { }
};

/**
 * This class listens to NodeStatus messages from other nodes and retrieves their unique ID if they are not
 * known to the allocator.
 */
class NodeDiscoverer : TimerBase
{
    typedef MethodBinder<NodeDiscoverer*, void (NodeDiscoverer::*)(const ServiceCallResult<protocol::GetNodeInfo>&)>
        GetNodeInfoResponseCallback;

    typedef MethodBinder<NodeDiscoverer*, void (NodeDiscoverer::*)(const ReceivedDataStructure<protocol::NodeStatus>&)>
        NodeStatusCallback;

    struct NodeData
    {
        uint32_t last_seen_uptime;
        uint8_t num_get_node_info_attempts;

        NodeData()
            : last_seen_uptime(0)
            , num_get_node_info_attempts(0)
        { }
    };

    typedef Map<NodeID, NodeData> NodeMap;

    /**
     * When this number of attempts has been made, the discoverer will give up and assume that the node
     * does not implement this service.
     */
    enum { MaxAttemptsToGetNodeInfo = 5 };

    enum { TimerPollIntervalMs = 170 }; // ~ ceil(500 ms service timeout / 3)

    /*
     * States
     */
    INodeDiscoveryHandler& handler_;
    IEventTracer& tracer_;

    BitSet<NodeID::Max + 1> committed_node_mask_;       ///< Nodes that are marked will not be queried
    NodeMap node_map_;

    ServiceClient<protocol::GetNodeInfo, GetNodeInfoResponseCallback> get_node_info_client_;
    Subscriber<protocol::NodeStatus, NodeStatusCallback> node_status_sub_;

    /*
     * Methods
     */
    void trace(TraceCode code, int64_t argument) { tracer_.onEvent(code, argument); }

    INode& getNode() { return node_status_sub_.getNode(); }

    void removeNode(const NodeID node_id)
    {
        node_map_.remove(node_id);
        trace(TraceDiscoveryNodeRemoved, node_id.get());
    }

    NodeID pickNextNodeToQuery() const
    {
        // This essentially means that we pick first available node. Remember that the map is unordered.
        const NodeMap::KVPair* const pair = node_map_.getByIndex(0);
        return (pair == UAVCAN_NULLPTR) ? NodeID() : pair->key;
    }

    bool needToQuery(NodeID node_id)
    {
        UAVCAN_ASSERT(node_id.isUnicast());

        /*
         * Fast check
         */
        if (committed_node_mask_[node_id.get()])
        {
            return false;
        }

        /*
         * Slow check - may involve full log search
         */
        const INodeDiscoveryHandler::NodeAwareness awareness = handler_.checkNodeAwareness(node_id);

        if (awareness == INodeDiscoveryHandler::NodeAwarenessUnknown)
        {
            return true;
        }
        else if (awareness == INodeDiscoveryHandler::NodeAwarenessKnownButNotCommitted)
        {
            removeNode(node_id);
            return false;
        }
        else if (awareness == INodeDiscoveryHandler::NodeAwarenessKnownAndCommitted)
        {
            trace(TraceDiscoveryCommitCacheUpdated, node_id.get());
            committed_node_mask_[node_id.get()] = true;
            removeNode(node_id);
            return false;
        }
        else
        {
            UAVCAN_ASSERT(0);
            return false;
        }
    }

    NodeID pickNextNodeToQueryAndCleanupMap()
    {
        NodeID node_id;
        do
        {
            node_id = pickNextNodeToQuery();
            if (node_id.isUnicast())
            {
                if (needToQuery(node_id))
                {
                    return node_id;
                }
                else
                {
                    removeNode(node_id);
                }
            }
        }
        while (node_id.isUnicast());
        return NodeID();
    }

    void finalizeNodeDiscovery(const UniqueID* unique_id_or_null, NodeID node_id)
    {
        trace(TraceDiscoveryNodeFinalized, node_id.get() | ((unique_id_or_null == UAVCAN_NULLPTR) ? 0U : 0x100U));
        removeNode(node_id);
        /*
         * It is paramount to check if the server is still interested to receive this data.
         * Otherwise, if the node appeared in the log while we were waiting for response, we'd end up with
         * duplicate node ID in the log.
         */
        if (needToQuery(node_id))
        {
            handler_.handleNewNodeDiscovery(unique_id_or_null, node_id);
        }
    }

    void handleGetNodeInfoResponse(const ServiceCallResult<protocol::GetNodeInfo>& result)
    {
        if (result.isSuccessful())
        {
            UAVCAN_TRACE("dynamic_node_id_server::NodeDiscoverer", "GetNodeInfo response from %d",
                         int(result.getCallID().server_node_id.get()));
            finalizeNodeDiscovery(&result.getResponse().hardware_version.unique_id, result.getCallID().server_node_id);
        }
        else
        {
            trace(TraceDiscoveryGetNodeInfoFailure, result.getCallID().server_node_id.get());

            NodeData* const data = node_map_.access(result.getCallID().server_node_id);
            if (data == UAVCAN_NULLPTR)
            {
                return;         // Probably it is a known node now
            }

            UAVCAN_TRACE("dynamic_node_id_server::NodeDiscoverer",
                         "GetNodeInfo request to %d has timed out, %d attempts",
                         int(result.getCallID().server_node_id.get()), int(data->num_get_node_info_attempts));
            data->num_get_node_info_attempts++;
            if (data->num_get_node_info_attempts >= MaxAttemptsToGetNodeInfo)
            {
                finalizeNodeDiscovery(UAVCAN_NULLPTR, result.getCallID().server_node_id);
                // Warning: data pointer is invalidated now
            }
        }
    }

    void handleTimerEvent(const TimerEvent&) override
    {
        if (get_node_info_client_.hasPendingCalls())
        {
            return;
        }

        const NodeID node_id = pickNextNodeToQueryAndCleanupMap();
        if (!node_id.isUnicast())
        {
            trace(TraceDiscoveryTimerStop, 0);
            stop();
            return;
        }

        if (!handler_.canDiscoverNewNodes())
        {
            return;     // Timer must continue to run in order to not stuck when it unlocks
        }

        trace(TraceDiscoveryGetNodeInfoRequest, node_id.get());

        UAVCAN_TRACE("dynamic_node_id_server::NodeDiscoverer", "Requesting GetNodeInfo from node %d",
                     int(node_id.get()));
        const int res = get_node_info_client_.call(node_id, protocol::GetNodeInfo::Request());
        if (res < 0)
        {
            getNode().registerInternalFailure("NodeDiscoverer GetNodeInfo call");
        }
    }

    void handleNodeStatus(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        if (!needToQuery(msg.getSrcNodeID()))
        {
            return;
        }

        NodeData* data = node_map_.access(msg.getSrcNodeID());
        if (data == UAVCAN_NULLPTR)
        {
            trace(TraceDiscoveryNewNodeFound, msg.getSrcNodeID().get());

            data = node_map_.insert(msg.getSrcNodeID(), NodeData());
            if (data == UAVCAN_NULLPTR)
            {
                getNode().registerInternalFailure("NodeDiscoverer OOM");
                return;
            }
        }
        UAVCAN_ASSERT(data != UAVCAN_NULLPTR);

        if (msg.uptime_sec < data->last_seen_uptime)
        {
            trace(TraceDiscoveryNodeRestartDetected, msg.getSrcNodeID().get());
            data->num_get_node_info_attempts = 0;
        }
        data->last_seen_uptime = msg.uptime_sec;

        if (!isRunning())
        {
            startPeriodic(MonotonicDuration::fromMSec(TimerPollIntervalMs));
            trace(TraceDiscoveryTimerStart, getPeriod().toUSec());
        }
    }

public:
    NodeDiscoverer(INode& node, IEventTracer& tracer, INodeDiscoveryHandler& handler)
        : TimerBase(node)
        , handler_(handler)
        , tracer_(tracer)
        , node_map_(node.getAllocator())
        , get_node_info_client_(node)
        , node_status_sub_(node)
    { }

    int init(const TransferPriority priority)
    {
        int res = get_node_info_client_.init(priority);
        if (res < 0)
        {
            return res;
        }
        get_node_info_client_.setCallback(
            GetNodeInfoResponseCallback(this, &NodeDiscoverer::handleGetNodeInfoResponse));

        res = node_status_sub_.start(NodeStatusCallback(this, &NodeDiscoverer::handleNodeStatus));
        if (res < 0)
        {
            return res;
        }

        // Note: the timer starts ad-hoc from the node status callback, not here.

        return 0;
    }

    /**
     * Returns true if there's at least one node with pending GetNodeInfo.
     */
    bool hasUnknownNodes() const { return !node_map_.isEmpty(); }

    /**
     * Returns number of nodes that are being queried at the moment.
     * This method is needed for testing and state visualization.
     */
    uint8_t getNumUnknownNodes() const { return static_cast<uint8_t>(node_map_.getSize()); }
};

}
}

#endif // Include guard
