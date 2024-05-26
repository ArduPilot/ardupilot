/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_SERVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_SERVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/raft_core.hpp>
#include <uavcan/protocol/dynamic_node_id_server/abstract_server.hpp>
#include <uavcan/protocol/dynamic_node_id_server/node_id_selector.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{
/**
 * This class implements the top-level allocation logic and server API.
 */
class UAVCAN_EXPORT Server : public AbstractServer
                           , IRaftLeaderMonitor
{
    struct UniqueIDLogPredicate
    {
        const UniqueID unique_id;

        UniqueIDLogPredicate(const UniqueID& uid)
            : unique_id(uid)
        { }

        bool operator()(const RaftCore::LogEntryInfo& info) const
        {
            return info.entry.unique_id == unique_id;
        }
    };

    struct NodeIDLogPredicate
    {
        const NodeID node_id;

        NodeIDLogPredicate(const NodeID& nid)
            : node_id(nid)
        { }

        bool operator()(const RaftCore::LogEntryInfo& info) const
        {
            return info.entry.node_id == node_id.get();
        }
    };

    /*
     * States
     */
    RaftCore raft_core_;

    /*
     * Methods of IAllocationRequestHandler
     */
    virtual bool canPublishFollowupAllocationResponse() const
    {
        /*
         * The server is allowed to publish follow-up allocation responses only if both conditions are met:
         * - The server is leader.
         * - The last allocation request has been completed successfully.
         *
         * Why second condition? Imagine a case when there's two Raft nodes that don't hear each other - A and B,
         * both of them are leaders (but only A can commit to the log, B is in a minor partition); then there's a
         * client X that can exchange with both leaders, and a client Y that can exchange only with A. Such a
         * situation can occur in case of a very unlikely failure of redundant interfaces.
         *
         * Both clients X and Y initially send a first-stage Allocation request; A responds to Y with a first-stage
         * response, whereas B responds to X. Both X and Y will issue a follow-up second-stage requests, which may
         * cause A to mix second-stage Allocation requests from different nodes, leading to reception of an invalid
         * unique ID. When both leaders receive full unique IDs (A will receive an invalid one, B will receive a valid
         * unique ID of X), only A will be able to make a commit, because B is in a minority. Since both clients were
         * unable to receive node ID values in this round, they will try again later.
         *
         * Now, in order to prevent B from disrupting client-server communication second time around, we introduce this
         * second restriction: the server cannot exchange with clients as long as its log contains uncommitted entries.
         *
         * Note that this restriction does not apply to allocation requests sent via CAN FD frames, as in this case
         * no follow-up responses are necessary. So only CAN FD can offer reliable Allocation exchange.
         */
        return raft_core_.isLeader() && raft_core_.areAllLogEntriesCommitted();
    }

    virtual void handleAllocationRequest(const UniqueID& unique_id, const NodeID preferred_node_id)
    {
        /*
         * Note that it is possible that the local node is not leader. We will still perform the log search
         * and try to find the node that requested allocation. If the node is found, response will be sent;
         * otherwise the request will be ignored because only leader can add new allocations.
         */
        const LazyConstructor<RaftCore::LogEntryInfo> result =
            raft_core_.traverseLogFromEndUntil(UniqueIDLogPredicate(unique_id));

         if (result.isConstructed())
         {
             if (result->committed)
             {
                 tryPublishAllocationResult(result->entry);
                 UAVCAN_TRACE("dynamic_node_id_server::distributed::Server",
                              "Allocation request served with existing allocation; node ID %d",
                              int(result->entry.node_id));
             }
             else
             {
                 UAVCAN_TRACE("dynamic_node_id_server::distributed::Server",
                              "Allocation request ignored - allocation exists but not committed yet; node ID %d",
                              int(result->entry.node_id));
             }
         }
         else
         {
             if (raft_core_.isLeader() && !node_discoverer_.hasUnknownNodes())
             {
                 allocateNewNode(unique_id, preferred_node_id);
             }
         }
    }

    /*
     * Methods of INodeDiscoveryHandler
     */
    virtual bool canDiscoverNewNodes() const
    {
        return raft_core_.isLeader();
    }

    virtual NodeAwareness checkNodeAwareness(NodeID node_id) const
    {
        const LazyConstructor<RaftCore::LogEntryInfo> result =
            raft_core_.traverseLogFromEndUntil(NodeIDLogPredicate(node_id));
        if (result.isConstructed())
        {
            return result->committed ? NodeAwarenessKnownAndCommitted : NodeAwarenessKnownButNotCommitted;
        }
        else
        {
            return NodeAwarenessUnknown;
        }
    }

    virtual void handleNewNodeDiscovery(const UniqueID* unique_id_or_null, NodeID node_id)
    {
        if (raft_core_.traverseLogFromEndUntil(NodeIDLogPredicate(node_id)).isConstructed())
        {
            UAVCAN_ASSERT(0);   // Such node is already known, the class that called this method should have known that
            return;
        }

        const UniqueID uid = (unique_id_or_null == UAVCAN_NULLPTR) ? UniqueID() : *unique_id_or_null;

        if (raft_core_.isLeader())
        {
            raft_core_.appendLog(uid, node_id);
        }
    }

    /*
     * Methods of IRaftLeaderMonitor
     */
    virtual void handleLogCommitOnLeader(const protocol::dynamic_node_id::server::Entry& entry)
    {
        /*
         * Maybe this node did not request allocation at all, we don't care, we publish anyway.
         */
        tryPublishAllocationResult(entry);
    }

    virtual void handleLocalLeadershipChange(bool local_node_is_leader)
    {
        if (!local_node_is_leader)
        {
            return;
        }

        const LazyConstructor<RaftCore::LogEntryInfo> result =
            raft_core_.traverseLogFromEndUntil(NodeIDLogPredicate(node_.getNodeID()));

        if (!result.isConstructed())
        {
            raft_core_.appendLog(getOwnUniqueID(), node_.getNodeID());
        }
    }

    /*
     * Private methods
     */
    bool isNodeIDTaken(const NodeID node_id) const
    {
        UAVCAN_TRACE("dynamic_node_id_server::distributed::Server",
                     "Testing if node ID %d is taken", int(node_id.get()));
        return raft_core_.traverseLogFromEndUntil(NodeIDLogPredicate(node_id));
    }

    void allocateNewNode(const UniqueID& unique_id, const NodeID preferred_node_id)
    {
        const NodeID allocated_node_id =
            NodeIDSelector<Server>(this, &Server::isNodeIDTaken).findFreeNodeID(preferred_node_id);
        if (!allocated_node_id.isUnicast())
        {
            UAVCAN_TRACE("dynamic_node_id_server::distributed::Server", "Request ignored - no free node ID left");
            return;
        }

        UAVCAN_TRACE("dynamic_node_id_server::distributed::Server", "New node ID allocated: %d",
                     int(allocated_node_id.get()));
        raft_core_.appendLog(unique_id, allocated_node_id);
    }

    void tryPublishAllocationResult(const protocol::dynamic_node_id::server::Entry& entry)
    {
        const int res = allocation_request_manager_.broadcastAllocationResponse(entry.unique_id, entry.node_id);
        if (res < 0)
        {
            tracer_.onEvent(TraceError, res);
            node_.registerInternalFailure("Dynamic allocation response");
        }
    }

public:
    Server(INode& node,
           IStorageBackend& storage,
           IEventTracer& tracer)
        : AbstractServer(node, tracer)
        , raft_core_(node, storage, tracer, *this)
    { }

    int init(const UniqueID& own_unique_id,
             const uint8_t cluster_size = ClusterManager::ClusterSizeUnknown,
             const TransferPriority priority = TransferPriority::OneHigherThanLowest)
    {
        /*
         * Initializing Raft core first, because the next step requires Log to be loaded
         */
        int res = raft_core_.init(cluster_size, priority);
        if (res < 0)
        {
            return res;
        }

        /*
         * Common logic
         */
        res = AbstractServer::init(own_unique_id, priority);
        if (res < 0)
        {
            return res;
        }

        /*
         * Making sure that the server is started with the same node ID
         */
        const LazyConstructor<RaftCore::LogEntryInfo> own_log_entry =
            raft_core_.traverseLogFromEndUntil(NodeIDLogPredicate(node_.getNodeID()));

        if (own_log_entry.isConstructed())
        {
            if (own_log_entry->entry.unique_id != getOwnUniqueID())
            {
                return -ErrInvalidConfiguration;
            }
        }

        return 0;
    }

    Log::Index getNumAllocations() const { return raft_core_.getNumAllocations(); }

    /**
     * These accessors are needed for debugging, visualization and testing.
     */
    const RaftCore& getRaftCore() const { return raft_core_; }
};

/**
 * This structure represents immediate state of the server.
 * It can be used for state visualization and debugging.
 */
struct StateReport
{
    uint8_t cluster_size;

    RaftCore::ServerState state;

    Log::Index last_log_index;
    Log::Index commit_index;

    Term last_log_term;
    Term current_term;

    NodeID voted_for;

    MonotonicTime last_activity_timestamp;
    MonotonicDuration randomized_timeout;

    uint8_t num_unknown_nodes;

    struct FollowerState
    {
        NodeID node_id;
        Log::Index next_index;
        Log::Index match_index;

        FollowerState()
            : next_index(0)
            , match_index(0)
        { }
    } followers[ClusterManager::MaxClusterSize - 1];

    StateReport(const Server& s)
        : cluster_size           (s.getRaftCore().getClusterManager().getClusterSize())
        , state                  (s.getRaftCore().getServerState())
        , last_log_index         (s.getRaftCore().getPersistentState().getLog().getLastIndex())
        , commit_index           (s.getRaftCore().getCommitIndex())
        , last_log_term          (0)    // See below
        , current_term           (s.getRaftCore().getPersistentState().getCurrentTerm())
        , voted_for              (s.getRaftCore().getPersistentState().getVotedFor())
        , last_activity_timestamp(s.getRaftCore().getLastActivityTimestamp())
        , randomized_timeout     (s.getRaftCore().getRandomizedTimeout())
        , num_unknown_nodes      (s.getNodeDiscoverer().getNumUnknownNodes())
    {
        const Entry* const e = s.getRaftCore().getPersistentState().getLog().getEntryAtIndex(last_log_index);
        UAVCAN_ASSERT(e != UAVCAN_NULLPTR);
        if (e != UAVCAN_NULLPTR)
        {
            last_log_term = e->term;
        }

        for (uint8_t i = 0; i < (cluster_size - 1U); i++)
        {
            const ClusterManager& mgr = s.getRaftCore().getClusterManager();
            const NodeID node_id = mgr.getRemoteServerNodeIDAtIndex(i);
            if (node_id.isUnicast())
            {
                followers[i].node_id     = node_id;
                followers[i].next_index  = mgr.getServerNextIndex(node_id);
                followers[i].match_index = mgr.getServerMatchIndex(node_id);
            }
        }
    }
};

}
}
}

#endif // Include guard
