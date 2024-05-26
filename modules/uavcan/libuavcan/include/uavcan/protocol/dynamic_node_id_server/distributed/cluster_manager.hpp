/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_CLUSTER_MANAGER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_SERVER_DISTRIBUTED_CLUSTER_MANAGER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/log.hpp>
#include <uavcan/protocol/dynamic_node_id_server/distributed/types.hpp>
#include <uavcan/protocol/dynamic_node_id_server/storage_marshaller.hpp>
#include <uavcan/protocol/dynamic_node_id_server/event.hpp>
// UAVCAN types
#include <uavcan/protocol/dynamic_node_id/server/Discovery.hpp>

namespace uavcan
{
namespace dynamic_node_id_server
{
namespace distributed
{
/**
 * This class maintains the cluster state.
 */
class ClusterManager : private TimerBase
{
public:
    enum { MaxClusterSize = Discovery::FieldTypes::known_nodes::MaxSize };

private:
    typedef MethodBinder<ClusterManager*,
                         void (ClusterManager::*)
                             (const ReceivedDataStructure<Discovery>&)>
        DiscoveryCallback;

    struct Server
    {
        NodeID node_id;
        Log::Index next_index;
        Log::Index match_index;

        Server()
            : next_index(0)
            , match_index(0)
        { }

        void resetIndices(const Log& log)
        {
            next_index = Log::Index(log.getLastIndex() + 1U);
            match_index = 0;
        }
    };

    IStorageBackend& storage_;
    IEventTracer& tracer_;
    const Log& log_;

    Subscriber<Discovery, DiscoveryCallback> discovery_sub_;
    mutable Publisher<Discovery> discovery_pub_;

    Server servers_[MaxClusterSize - 1];   ///< Minus one because the local server is not listed there.

    uint8_t cluster_size_;
    uint8_t num_known_servers_;

    static IStorageBackend::String getStorageKeyForClusterSize() { return "cluster_size"; }

    INode&       getNode()       { return discovery_sub_.getNode(); }
    const INode& getNode() const { return discovery_sub_.getNode(); }

    const Server* findServer(NodeID node_id) const { return const_cast<ClusterManager*>(this)->findServer(node_id); }
    Server*       findServer(NodeID node_id)
    {
        for (uint8_t i = 0; i < num_known_servers_; i++)
        {
            UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
            if (servers_[i].node_id == node_id)
            {
                return &servers_[i];
            }
        }
        return UAVCAN_NULLPTR;
    }

    virtual void handleTimerEvent(const TimerEvent&)
    {
        UAVCAN_ASSERT(num_known_servers_ < cluster_size_);

        tracer_.onEvent(TraceRaftDiscoveryBroadcast, num_known_servers_);

        /*
         * Filling the message
         */
        Discovery msg;
        msg.configured_cluster_size = cluster_size_;

        msg.known_nodes.push_back(getNode().getNodeID().get());     // Putting ourselves at index 0

        for (uint8_t i = 0; i < num_known_servers_; i++)
        {
            UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
            msg.known_nodes.push_back(servers_[i].node_id.get());
        }

        UAVCAN_ASSERT(msg.known_nodes.size() == (num_known_servers_ + 1));

        /*
         * Broadcasting
         */
        UAVCAN_TRACE("dynamic_node_id_server::distributed::ClusterManager",
                     "Broadcasting Discovery message; known nodes: %d of %d",
                     int(msg.known_nodes.size()), int(cluster_size_));

        const int res = discovery_pub_.broadcast(msg);
        if (res < 0)
        {
            UAVCAN_TRACE("dynamic_node_id_server::distributed::ClusterManager", "Discovery broadcst failed: %d", res);
            getNode().registerInternalFailure("Raft discovery broadcast");
        }

        /*
         * Termination condition
         */
        if (isClusterDiscovered())
        {
            UAVCAN_TRACE("dynamic_node_id_server::distributed::ClusterManager",
                         "Discovery broadcasting timer stopped");
            stop();
        }
    }

    void handleDiscovery(const ReceivedDataStructure<Discovery>& msg)
    {
        tracer_.onEvent(TraceRaftDiscoveryReceived, msg.getSrcNodeID().get());

        /*
         * Validating cluster configuration
         * If there's a case of misconfiguration, the message will be ignored.
         */
        if (msg.configured_cluster_size != cluster_size_)
        {
            tracer_.onEvent(TraceRaftBadClusterSizeReceived, msg.configured_cluster_size);
            getNode().registerInternalFailure("Bad Raft cluster size");
            return;
        }

        /*
         * Updating the set of known servers
         */
        for (uint8_t i = 0; i < msg.known_nodes.size(); i++)
        {
            if (isClusterDiscovered())
            {
                break;
            }

            const NodeID node_id(msg.known_nodes[i]);
            if (node_id.isUnicast() && !isKnownServer(node_id))
            {
                addServer(node_id);
            }
        }

        /*
         * Publishing a new Discovery request if the publishing server needs to learn about more servers.
         */
        if (msg.configured_cluster_size > msg.known_nodes.size())
        {
            startDiscoveryPublishingTimerIfNotRunning();
        }
    }

    void startDiscoveryPublishingTimerIfNotRunning()
    {
        if (!isRunning())
        {
            startPeriodic(MonotonicDuration::fromMSec(Discovery::BROADCASTING_PERIOD_MS));
        }
    }

public:
    enum { ClusterSizeUnknown = 0 };

    /**
     * @param node          Needed to publish and subscribe to Discovery message
     * @param storage       Needed to read the cluster size parameter from the storage
     * @param log           Needed to initialize nextIndex[] values after elections
     */
    ClusterManager(INode& node, IStorageBackend& storage, const Log& log, IEventTracer& tracer)
        : TimerBase(node)
        , storage_(storage)
        , tracer_(tracer)
        , log_(log)
        , discovery_sub_(node)
        , discovery_pub_(node)
        , cluster_size_(0)
        , num_known_servers_(0)
    { }

    /**
     * If cluster_size is set to ClusterSizeUnknown, the class will try to read this parameter from the
     * storage backend using key 'cluster_size'.
     * Returns negative error code.
     */
    int init(const uint8_t init_cluster_size, const TransferPriority priority)
    {
        /*
         * Figuring out the cluster size
         */
        if (init_cluster_size == ClusterSizeUnknown)
        {
            // Reading from the storage
            StorageMarshaller io(storage_);
            uint32_t value = 0;
            int res = io.get(getStorageKeyForClusterSize(), value);
            if (res < 0)
            {
                UAVCAN_TRACE("dynamic_node_id_server::distributed::ClusterManager",
                             "Cluster size is neither configured nor stored in the storage");
                return res;
            }
            if ((value == 0) || (value > MaxClusterSize))
            {
                UAVCAN_TRACE("dynamic_node_id_server::distributed::ClusterManager", "Cluster size is invalid");
                return -ErrInvalidConfiguration;
            }
            cluster_size_ = static_cast<uint8_t>(value);
        }
        else
        {
            if ((init_cluster_size == 0) || (init_cluster_size > MaxClusterSize))
            {
                return -ErrInvalidParam;
            }
            cluster_size_ = init_cluster_size;

            // Writing the storage
            StorageMarshaller io(storage_);
            uint32_t value = init_cluster_size;
            int res = io.setAndGetBack(getStorageKeyForClusterSize(), value);
            if ((res < 0) || (value != init_cluster_size))
            {
                UAVCAN_TRACE("dynamic_node_id_server::distributed::ClusterManager", "Failed to store cluster size");
                return -ErrFailure;
            }
        }

        tracer_.onEvent(TraceRaftClusterSizeInited, cluster_size_);

        UAVCAN_ASSERT(cluster_size_ > 0);
        UAVCAN_ASSERT(cluster_size_ <= MaxClusterSize);

        /*
         * Initializing pub/sub and timer
         */
        int res = discovery_pub_.init(priority);
        if (res < 0)
        {
            return res;
        }

        res = discovery_sub_.start(DiscoveryCallback(this, &ClusterManager::handleDiscovery));
        if (res < 0)
        {
            return res;
        }

        startDiscoveryPublishingTimerIfNotRunning();

        /*
         * Misc
         */
        resetAllServerIndices();
        return 0;
    }

    /**
     * Adds once server regardless of the discovery logic.
     */
    void addServer(NodeID node_id)
    {
        UAVCAN_ASSERT((num_known_servers_ + 1) < MaxClusterSize);
        if (!isKnownServer(node_id) && node_id.isUnicast())
        {
            tracer_.onEvent(TraceRaftNewServerDiscovered, node_id.get());
            servers_[num_known_servers_].node_id = node_id;
            servers_[num_known_servers_].resetIndices(log_);
            num_known_servers_ = static_cast<uint8_t>(num_known_servers_ + 1U);
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * Whether such server has been discovered.
     */
    bool isKnownServer(NodeID node_id) const
    {
        if (node_id == getNode().getNodeID())
        {
            return true;
        }
        for (uint8_t i = 0; i < num_known_servers_; i++)
        {
            UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
            UAVCAN_ASSERT(servers_[i].node_id != getNode().getNodeID());
            if (servers_[i].node_id == node_id)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * An invalid node ID will be returned if there's no such server.
     * The local server is not listed there.
     */
    NodeID getRemoteServerNodeIDAtIndex(uint8_t index) const
    {
        if (index < num_known_servers_)
        {
            return servers_[index].node_id;
        }
        return NodeID();
    }

    /**
     * See next_index[] in Raft paper.
     */
    Log::Index getServerNextIndex(NodeID server_node_id) const
    {
        const Server* const s = findServer(server_node_id);
        if (s != UAVCAN_NULLPTR)
        {
            return s->next_index;
        }
        UAVCAN_ASSERT(0);
        return 0;
    }

    void incrementServerNextIndexBy(NodeID server_node_id, Log::Index increment)
    {
        Server* const s = findServer(server_node_id);
        if (s != UAVCAN_NULLPTR)
        {
            s->next_index = Log::Index(s->next_index + increment);
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    void decrementServerNextIndex(NodeID server_node_id)
    {
        Server* const s = findServer(server_node_id);
        if (s != UAVCAN_NULLPTR)
        {
            s->next_index--;
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * See match_index[] in Raft paper.
     */
    Log::Index getServerMatchIndex(NodeID server_node_id) const
    {
        const Server* const s = findServer(server_node_id);
        if (s != UAVCAN_NULLPTR)
        {
            return s->match_index;
        }
        UAVCAN_ASSERT(0);
        return 0;
    }

    void setServerMatchIndex(NodeID server_node_id, Log::Index match_index)
    {
        Server* const s = findServer(server_node_id);
        if (s != UAVCAN_NULLPTR)
        {
            s->match_index = match_index;
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * This method must be called when the current server becomes leader.
     */
    void resetAllServerIndices()
    {
        for (uint8_t i = 0; i < num_known_servers_; i++)
        {
            UAVCAN_ASSERT(servers_[i].node_id.isUnicast());
            servers_[i].resetIndices(log_);
        }
    }

    /**
     * Number of known servers can only grow, and it never exceeds the cluster size value.
     * This number does not include the local server.
     */
    uint8_t getNumKnownServers() const { return num_known_servers_; }

    /**
     * Cluster size and quorum size are constant.
     */
    uint8_t getClusterSize() const { return cluster_size_; }
    uint8_t getQuorumSize() const { return static_cast<uint8_t>(cluster_size_ / 2U + 1U); }

    bool isClusterDiscovered() const { return num_known_servers_ == (cluster_size_ - 1); }
};

}
}
}

#endif // Include guard
