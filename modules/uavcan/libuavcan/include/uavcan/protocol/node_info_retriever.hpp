/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_NODE_INFO_RETRIEVER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_NODE_INFO_RETRIEVER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>
#include <uavcan/util/multiset.hpp>
#include <uavcan/node/service_client.hpp>
#include <uavcan/node/timer.hpp>
#include <uavcan/protocol/node_status_monitor.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>

namespace uavcan
{
/**
 * Classes that need to receive GetNodeInfo responses should implement this interface.
 */
class UAVCAN_EXPORT INodeInfoListener
{
public:
    /**
     * Called when a response to GetNodeInfo request is received. This happens shortly after the node restarts or
     * becomes online for the first time.
     * @param node_id   Node ID of the node
     * @param response  Node info struct
     */
    virtual void handleNodeInfoRetrieved(NodeID node_id, const protocol::GetNodeInfo::Response& node_info) = 0;

    /**
     * Called when the retriever decides that the node does not support the GetNodeInfo service.
     * This method will never be called if the number of attempts is unlimited.
     */
    virtual void handleNodeInfoUnavailable(NodeID node_id) = 0;

    /**
     * This call is routed directly from @ref NodeStatusMonitor.
     * Default implementation does nothing.
     * @param event     Node status change event
     */
    virtual void handleNodeStatusChange(const NodeStatusMonitor::NodeStatusChangeEvent& event)
    {
        (void)event;
    }

    /**
     * This call is routed directly from @ref NodeStatusMonitor.
     * Default implementation does nothing.
     * @param msg       Node status message
     */
    virtual void handleNodeStatusMessage(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        (void)msg;
    }

    virtual ~INodeInfoListener() { }
};

/**
 * This class automatically retrieves a response to GetNodeInfo once a node appears online or restarts.
 * It does a number of attempts in case if there's a communication failure before assuming that the node does not
 * implement the GetNodeInfo service. All parameters are pre-configured with sensible default values that should fit
 * virtually any use case, but they can be overriden if needed - refer to the setter methods below for details.
 *
 * Defaults are pre-configured so that the class is able to query 123 nodes (node ID 1..125, where 1 is our local
 * node and 1 is one node that implements GetNodeInfo service, hence 123) of which none implements GetNodeInfo
 * service in under 5 seconds. The 5 second limitation is imposed by UAVCAN-compatible bootloaders, which are
 * unlikely to wait for more than that before continuing to boot. In case if this default value is not appropriate
 * for the end application, the request interval can be overriden via @ref setRequestInterval().
 *
 * Following the above explained requirements, the default request interval is defined as follows:
 *      request interval [ms] = floor(5000 [ms] bootloader timeout / 123 nodes)
 * Which yields 40 ms.
 *
 * Given default service timeout 1000 ms and the defined above request frequency 40 ms, the maximum number of
 * concurrent requests will be:
 *      max concurrent requests = ceil(1000 [ms] timeout / 40 [ms] request interval)
 * Which yields 25 requests.
 *
 * Keep the above equations in mind when changing the default request interval.
 *
 * Obviously, if all calls are completing in under (request interval), the number of concurrent requests will never
 * exceed one. This is actually the most likely scenario.
 *
 * Note that all nodes are queried in a round-robin fashion, regardless of their uptime, number of requests made, etc.
 *
 * Events from this class can be routed to many listeners, @ref INodeInfoListener.
 */
class UAVCAN_EXPORT NodeInfoRetriever : public NodeStatusMonitor
                                      , TimerBase
{
public:
    enum { MaxNumRequestAttempts = 254 };
    enum { UnlimitedRequestAttempts = 0 };

private:
    typedef MethodBinder<NodeInfoRetriever*,
                         void (NodeInfoRetriever::*)(const ServiceCallResult<protocol::GetNodeInfo>&)>
            GetNodeInfoResponseCallback;

    struct Entry
    {
        uint32_t uptime_sec;
        uint8_t num_attempts_made;
        bool request_needed;                    ///< Always false for unknown nodes
        bool updated_since_last_attempt;        ///< Always false for unknown nodes

        Entry()
            : uptime_sec(0)
            , num_attempts_made(0)
            , request_needed(false)
            , updated_since_last_attempt(false)
        {
#if UAVCAN_DEBUG
            StaticAssert<sizeof(Entry) <= 8>::check();
#endif
        }
    };

    struct NodeInfoRetrievedHandlerCaller
    {
        const NodeID node_id;
        const protocol::GetNodeInfo::Response& node_info;

        NodeInfoRetrievedHandlerCaller(NodeID arg_node_id, const protocol::GetNodeInfo::Response& arg_node_info)
            : node_id(arg_node_id)
            , node_info(arg_node_info)
        { }

        bool operator()(INodeInfoListener* key)
        {
            UAVCAN_ASSERT(key != UAVCAN_NULLPTR);
            key->handleNodeInfoRetrieved(node_id, node_info);
            return false;
        }
    };

    template <typename Event>
    struct GenericHandlerCaller
    {
        void (INodeInfoListener::* const method)(Event);
        Event event;

        GenericHandlerCaller(void (INodeInfoListener::*arg_method)(Event), Event arg_event)
            : method(arg_method)
            , event(arg_event)
        { }

        bool operator()(INodeInfoListener* key)
        {
            UAVCAN_ASSERT(key != UAVCAN_NULLPTR);
            (key->*method)(event);
            return false;
        }
    };

    enum { DefaultNumRequestAttempts = 16 };
    enum { DefaultTimerIntervalMSec = 40 };  ///< Read explanation in the class documentation

    /*
     * State
     */
    Entry entries_[NodeID::Max];  // [1, NodeID::Max]

    Multiset<INodeInfoListener*> listeners_;

    ServiceClient<protocol::GetNodeInfo, GetNodeInfoResponseCallback> get_node_info_client_;

    MonotonicDuration request_interval_;

    mutable uint8_t last_picked_node_;

    uint8_t num_attempts_;

    /*
     * Methods
     */
    const Entry& getEntry(NodeID node_id) const { return const_cast<NodeInfoRetriever*>(this)->getEntry(node_id); }
    Entry&       getEntry(NodeID node_id)
    {
        if (node_id.get() < 1 || node_id.get() > NodeID::Max)
        {
            handleFatalError("NodeInfoRetriever NodeID");
        }
        return entries_[node_id.get() - 1];
    }

    void startTimerIfNotRunning()
    {
        if (!TimerBase::isRunning())
        {
            TimerBase::startPeriodic(request_interval_);
            UAVCAN_TRACE("NodeInfoRetriever", "Timer started, interval %s sec", request_interval_.toString().c_str());
        }
    }

    NodeID pickNextNodeToQuery(bool& out_at_least_one_request_needed) const
    {
        out_at_least_one_request_needed = false;

        for (unsigned iter_cnt_ = 0; iter_cnt_ < (sizeof(entries_) / sizeof(entries_[0])); iter_cnt_++) // Round-robin
        {
            last_picked_node_++;
            if (last_picked_node_ > NodeID::Max)
            {
                last_picked_node_ = 1;
            }
            UAVCAN_ASSERT((last_picked_node_ >= 1) &&
                          (last_picked_node_ <= NodeID::Max));

            const Entry& entry = getEntry(last_picked_node_);

            if (entry.request_needed)
            {
                out_at_least_one_request_needed = true;

                if (entry.updated_since_last_attempt &&
                    !get_node_info_client_.hasPendingCallToServer(last_picked_node_))
                {
                    UAVCAN_TRACE("NodeInfoRetriever", "Next node to query: %d", int(last_picked_node_));
                    return NodeID(last_picked_node_);
                }
            }
        }

        return NodeID();        // No node could be found
    }

    virtual void handleTimerEvent(const TimerEvent&)
    {
        bool at_least_one_request_needed = false;
        const NodeID next = pickNextNodeToQuery(at_least_one_request_needed);

        if (next.isUnicast())
        {
            UAVCAN_ASSERT(at_least_one_request_needed);
            getEntry(next).updated_since_last_attempt = false;
            const int res = get_node_info_client_.call(next, protocol::GetNodeInfo::Request());
            if (res < 0)
            {
                get_node_info_client_.getNode().registerInternalFailure("NodeInfoRetriever GetNodeInfo call");
            }
        }
        else
        {
            if (!at_least_one_request_needed)
            {
                TimerBase::stop();
                UAVCAN_TRACE("NodeInfoRetriever", "Timer stopped");
            }
        }
    }

    virtual void handleNodeStatusChange(const NodeStatusChangeEvent& event)
    {
        const bool was_offline = !event.was_known ||
                                 (event.old_status.mode == protocol::NodeStatus::MODE_OFFLINE);

        const bool offline_now = event.status.mode == protocol::NodeStatus::MODE_OFFLINE;

        if (was_offline || offline_now)
        {
            Entry& entry = getEntry(event.node_id);

            entry.request_needed = !offline_now;
            entry.num_attempts_made = 0;

            UAVCAN_TRACE("NodeInfoRetriever", "Offline status change: node ID %d, request needed: %d",
                         int(event.node_id.get()), int(entry.request_needed));

            if (entry.request_needed)
            {
                startTimerIfNotRunning();
            }
        }

        listeners_.forEach(
            GenericHandlerCaller<const NodeStatusChangeEvent&>(&INodeInfoListener::handleNodeStatusChange, event));
    }

    virtual void handleNodeStatusMessage(const ReceivedDataStructure<protocol::NodeStatus>& msg)
    {
        Entry& entry = getEntry(msg.getSrcNodeID());

        if (msg.uptime_sec < entry.uptime_sec)
        {
            entry.request_needed = true;
            entry.num_attempts_made = 0;

            startTimerIfNotRunning();
        }
        entry.uptime_sec = msg.uptime_sec;
        entry.updated_since_last_attempt = true;

        listeners_.forEach(GenericHandlerCaller<const ReceivedDataStructure<protocol::NodeStatus>&>(
            &INodeInfoListener::handleNodeStatusMessage, msg));
    }

    void handleGetNodeInfoResponse(const ServiceCallResult<protocol::GetNodeInfo>& result)
    {
        Entry& entry = getEntry(result.getCallID().server_node_id);

        if (result.isSuccessful())
        {
            /*
             * Updating the uptime here allows to properly handle a corner case where the service response arrives
             * after the device has restarted and published its new NodeStatus (although it's unlikely to happen).
             */
            entry.uptime_sec = result.getResponse().status.uptime_sec;
            entry.request_needed = false;
            listeners_.forEach(NodeInfoRetrievedHandlerCaller(result.getCallID().server_node_id,
                                                              result.getResponse()));
        }
        else
        {
            if (num_attempts_ != UnlimitedRequestAttempts)
            {
                entry.num_attempts_made++;
                if (entry.num_attempts_made >= num_attempts_)
                {
                    entry.request_needed = false;
                    listeners_.forEach(GenericHandlerCaller<NodeID>(&INodeInfoListener::handleNodeInfoUnavailable,
                                                                    result.getCallID().server_node_id));
                }
            }
        }
    }

public:
    NodeInfoRetriever(INode& node)
        : NodeStatusMonitor(node)
        , TimerBase(node)
        , listeners_(node.getAllocator())
        , get_node_info_client_(node)
        , request_interval_(MonotonicDuration::fromMSec(DefaultTimerIntervalMSec))
        , last_picked_node_(1)
        , num_attempts_(DefaultNumRequestAttempts)
    { }

    /**
     * Starts the retriever.
     * Destroy the object to stop it.
     * Returns negative error code.
     */
    int start(const TransferPriority priority = TransferPriority::OneHigherThanLowest)
    {
        int res = NodeStatusMonitor::start();
        if (res < 0)
        {
            return res;
        }

        res = get_node_info_client_.init(priority);
        if (res < 0)
        {
            return res;
        }
        get_node_info_client_.setCallback(GetNodeInfoResponseCallback(this,
                                                                      &NodeInfoRetriever::handleGetNodeInfoResponse));
        // Note: the timer will be started ad-hoc
        return 0;
    }

    /**
     * This method forces the class to re-request uavcan.protocol.GetNodeInfo from all nodes as if they
     * have just appeared in the network.
     */
    void invalidateAll()
    {
        NodeStatusMonitor::forgetAllNodes();
        get_node_info_client_.cancelAllCalls();

        for (unsigned i = 0; i < (sizeof(entries_) / sizeof(entries_[0])); i++)
        {
            entries_[i] = Entry();
        }
        // It is not necessary to reset the last picked node index
    }

    /**
     * Adds one listener. Does nothing if such listener already exists.
     * May return -ErrMemory if there's no space to add the listener.
     */
    int addListener(INodeInfoListener* listener)
    {
        if (listener != UAVCAN_NULLPTR)
        {
            removeListener(listener);
            return (UAVCAN_NULLPTR == listeners_.emplace(listener)) ? -ErrMemory : 0;
        }
        else
        {
            return -ErrInvalidParam;
        }
    }

    /**
     * Removes the listener.
     * If the listener was not registered, nothing will be done.
     */
    void removeListener(INodeInfoListener* listener)
    {
        if (listener != UAVCAN_NULLPTR)
        {
            listeners_.removeAll(listener);
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    unsigned getNumListeners() const { return listeners_.getSize(); }

    /**
     * Number of attempts to retrieve GetNodeInfo response before giving up on the assumption that the service is
     * not implemented.
     * Zero is a special value that can be used to set unlimited number of attempts, @ref UnlimitedRequestAttempts.
     */
    uint8_t getNumRequestAttempts() const { return num_attempts_; }
    void setNumRequestAttempts(const uint8_t num)
    {
        num_attempts_ = min(static_cast<uint8_t>(MaxNumRequestAttempts), num);
    }

    /**
     * Request interval also implicitly defines the maximum number of concurrent requests.
     * Read the class documentation for details.
     */
    MonotonicDuration getRequestInterval() const { return request_interval_; }
    void setRequestInterval(const MonotonicDuration interval)
    {
        if (interval.isPositive())
        {
            request_interval_ = interval;
            if (TimerBase::isRunning())
            {
                TimerBase::startPeriodic(request_interval_);
            }
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    /**
     * These methods are needed mostly for testing.
     */
    bool isRetrievingInProgress() const { return TimerBase::isRunning(); }

    uint8_t getNumPendingRequests() const
    {
        const unsigned num = get_node_info_client_.getNumPendingCalls();
        UAVCAN_ASSERT(num <= 0xFF);
        return static_cast<uint8_t>(num);
    }
};

}

#endif // Include guard
