/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_NODE_SERVICE_CLIENT_HPP_INCLUDED
#define UAVCAN_NODE_SERVICE_CLIENT_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/util/multiset.hpp>
#include <uavcan/node/generic_publisher.hpp>
#include <uavcan/node/generic_subscriber.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <functional>
#endif

namespace uavcan
{
/**
 * This struct describes a pending service call.
 * Refer to @ref ServiceClient to learn more about service calls.
 */
struct ServiceCallID
{
    NodeID server_node_id;
    TransferID transfer_id;

    ServiceCallID() { }

    ServiceCallID(NodeID arg_server_node_id, TransferID arg_transfer_id)
        : server_node_id(arg_server_node_id)
        , transfer_id(arg_transfer_id)
    { }

    bool operator==(const ServiceCallID rhs) const
    {
        return (rhs.server_node_id == server_node_id) &&
               (rhs.transfer_id == transfer_id);
    }

    bool isValid() const { return server_node_id.isUnicast(); }
};

/**
 * Object of this type will be returned to the application as a result of service call.
 * Note that application ALWAYS gets this result, even when it times out or fails because of some other reason.
 * The class is made noncopyable because it keeps a reference to a stack-allocated object.
 */
template <typename DataType>
class UAVCAN_EXPORT ServiceCallResult : Noncopyable
{
public:
    typedef ReceivedDataStructure<typename DataType::Response> ResponseFieldType;

    enum Status { Success, ErrorTimeout };

private:
    const Status status_;               ///< Whether successful or not. Failure to decode the response causes timeout.
    ServiceCallID call_id_;             ///< Identifies the call
    ResponseFieldType& response_;       ///< Returned data structure. Value undefined if the service call has failed.

public:
    ServiceCallResult(Status arg_status, ServiceCallID arg_call_id, ResponseFieldType& arg_response)
        : status_(arg_status)
        , call_id_(arg_call_id)
        , response_(arg_response)
    {
        UAVCAN_ASSERT(call_id_.isValid());
        UAVCAN_ASSERT((status_ == Success) || (status_ == ErrorTimeout));
    }

    /**
     * Shortcut to quickly check whether the call was successful.
     */
    bool isSuccessful() const { return status_ == Success; }

    Status getStatus() const { return status_; }

    ServiceCallID getCallID() const { return call_id_; }

    /**
     * Returned reference points to a stack-allocated object.
     */
    const ResponseFieldType& getResponse() const { return response_; }
    ResponseFieldType& getResponse() { return response_; }
};

/**
 * This operator neatly prints the service call result prepended with extra data like Server Node ID.
 * The extra data will be represented as YAML comment.
 */
template <typename Stream, typename DataType>
static Stream& operator<<(Stream& s, const ServiceCallResult<DataType>& scr)
{
    s << "# Service call result [" << DataType::getDataTypeFullName() << "] "
      << (scr.isSuccessful() ? "OK" : "FAILURE")
      << " server_node_id=" << int(scr.getCallID().server_node_id.get())
      << " tid=" << int(scr.getCallID().transfer_id.get()) << "\n";
    if (scr.isSuccessful())
    {
        s << scr.getResponse();
    }
    else
    {
        s << "# (no data)";
    }
    return s;
}

/**
 * Do not use directly.
 */
class ServiceClientBase : protected ITransferAcceptanceFilter
                        , protected DeadlineHandler
{
    const DataTypeDescriptor* data_type_descriptor_;  ///< This will be initialized at the time of first call

protected:
    class CallState : DeadlineHandler
    {
        ServiceClientBase& owner_;
        const ServiceCallID id_;
        bool timed_out_;

        virtual void handleDeadline(MonotonicTime) override;

    public:
        CallState(INode& node, ServiceClientBase& owner, ServiceCallID call_id)
            : DeadlineHandler(node.getScheduler())
            , owner_(owner)
            , id_(call_id)
            , timed_out_(false)
        {
            UAVCAN_ASSERT(id_.isValid());
            DeadlineHandler::startWithDelay(owner_.request_timeout_);
        }

        ServiceCallID getCallID() const { return id_; }

        bool hasTimedOut() const { return timed_out_; }

        static bool hasTimedOutPredicate(const CallState& cs) { return cs.hasTimedOut(); }

        bool operator==(const CallState& rhs) const
        {
            return (&owner_ == &rhs.owner_) && (id_ == rhs.id_);
        }
    };

    struct CallStateMatchingPredicate
    {
        const ServiceCallID id;
        CallStateMatchingPredicate(ServiceCallID reference) : id(reference) { }
        bool operator()(const CallState& state) const { return (state.getCallID() == id) && !state.hasTimedOut(); }
    };

    struct ServerSearchPredicate
    {
        const NodeID server_node_id;
        ServerSearchPredicate(NodeID nid) : server_node_id(nid) { }
        bool operator()(const CallState& state) const { return state.getCallID().server_node_id == server_node_id; }
    };

    MonotonicDuration request_timeout_;

    ServiceClientBase(INode& node)
        : DeadlineHandler(node.getScheduler())
        , data_type_descriptor_(UAVCAN_NULLPTR)
        , request_timeout_(getDefaultRequestTimeout())
    { }

    virtual ~ServiceClientBase() { }

    int prepareToCall(INode& node, const char* dtname, NodeID server_node_id, ServiceCallID& out_call_id);

public:
    /**
     * It's not recommended to override default timeouts.
     * Change of this value will not affect pending calls.
     */
    static MonotonicDuration getDefaultRequestTimeout() { return MonotonicDuration::fromMSec(1000); }
    static MonotonicDuration getMinRequestTimeout() { return MonotonicDuration::fromMSec(10); }
    static MonotonicDuration getMaxRequestTimeout() { return MonotonicDuration::fromMSec(60000); }
};

/**
 * Use this class to invoke services on remote nodes.
 *
 * This class can manage multiple concurrent calls to the same or different remote servers. Number of concurrent
 * calls is limited only by amount of available pool memory.
 *
 * Note that the reference passed to the callback points to a stack-allocated object, which means that the
 * reference invalidates once the callback returns. If you want to use this object after the callback execution,
 * you need to copy it somewhere.
 *
 * Note that by default, service client objects use lower priority than message publishers. Use @ref setPriority()
 * to override the default if necessary.
 *
 * @tparam DataType_        Service data type.
 *
 * @tparam Callback_        Service response will be delivered through the callback of this type.
 *                          In C++11 mode this type defaults to std::function<>.
 *                          In C++03 mode this type defaults to a plain function pointer; use binder to
 *                          call member functions as callbacks.
 */
template <typename DataType_,
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
          typename Callback_ = std::function<void (const ServiceCallResult<DataType_>&)>
#else
          typename Callback_ = void (*)(const ServiceCallResult<DataType_>&)
#endif
          >
class UAVCAN_EXPORT ServiceClient
    : public GenericSubscriber<DataType_,
                               typename DataType_::Response, TransferListenerWithFilter>
    , public ServiceClientBase
{
public:
    typedef DataType_ DataType;
    typedef typename DataType::Request RequestType;
    typedef typename DataType::Response ResponseType;
    typedef ServiceCallResult<DataType> ServiceCallResultType;
    typedef Callback_ Callback;

private:
    typedef ServiceClient<DataType, Callback> SelfType;
    typedef GenericPublisher<DataType, RequestType> PublisherType;
    typedef GenericSubscriber<DataType, ResponseType, TransferListenerWithFilter> SubscriberType;

    typedef Multiset<CallState> CallRegistry;

    struct TimeoutCallbackCaller
    {
        ServiceClient& owner;

        TimeoutCallbackCaller(ServiceClient& arg_owner) : owner(arg_owner) { }

        void operator()(const CallState& state)
        {
            if (state.hasTimedOut())
            {
                UAVCAN_TRACE("ServiceClient::TimeoutCallbackCaller", "Timeout from nid=%d, tid=%d, dtname=%s",
                             int(state.getCallID().server_node_id.get()), int(state.getCallID().transfer_id.get()),
                             DataType::getDataTypeFullName());

                typename SubscriberType::ReceivedDataStructureSpec rx_struct; // Default-initialized

                ServiceCallResultType result(ServiceCallResultType::ErrorTimeout, state.getCallID(),
                                             rx_struct);    // Mutable!

                owner.invokeCallback(result);
            }
        }
    };

    CallRegistry call_registry_;

    PublisherType publisher_;
    Callback callback_;

    virtual bool shouldAcceptFrame(const RxFrame& frame) const override; // Called from the transfer listener

    void invokeCallback(ServiceCallResultType& result);

    virtual void handleReceivedDataStruct(ReceivedDataStructure<ResponseType>& response) override;

    virtual void handleDeadline(MonotonicTime) override;

    int addCallState(ServiceCallID call_id);

public:
    /**
     * @param node      Node instance this client will be registered with.
     * @param callback  Callback instance. Optional, can be assigned later.
     */
    explicit ServiceClient(INode& node, const Callback& callback = Callback(), bool force_std_can = false)
        : SubscriberType(node)
        , ServiceClientBase(node)
        , call_registry_(node.getAllocator())
        , publisher_(node, force_std_can, getDefaultRequestTimeout())
        , callback_(callback)
    {
        setPriority(TransferPriority::MiddleLower);
        setRequestTimeout(getDefaultRequestTimeout());
#if UAVCAN_DEBUG
        UAVCAN_ASSERT(getRequestTimeout() == getDefaultRequestTimeout());  // Making sure default values are OK
#endif
    }

    virtual ~ServiceClient() { cancelAllCalls(); }

    /**
     * Shall be called before first use.
     * Returns negative error code.
     */
    int init()
    {
        return publisher_.init();
    }

    /**
     * Shall be called before first use.
     * This overload allows to set the priority, otherwise it's the same.
     * Returns negative error code.
     */
    int init(TransferPriority priority)
    {
        return publisher_.init(priority);
    }

    /**
     * Performs non-blocking service call.
     * This method transmits the service request and returns immediately.
     *
     * Service response will be delivered into the application via the callback.
     * Note that the callback will ALWAYS be called even if the service call times out; the
     * actual result of the call (success/failure) will be passed to the callback as well.
     *
     * Returns negative error code.
     */
    int call(NodeID server_node_id, const RequestType& request);

    /**
     * Same as plain @ref call() above, but this overload also returns the call ID of the new call.
     * The call ID structure can be used to cancel this request later if needed.
     */
    int call(NodeID server_node_id, const RequestType& request, ServiceCallID& out_call_id);

    /**
     * Cancels certain call referred via call ID structure.
     */
    void cancelCall(ServiceCallID call_id);

    /**
     * Cancels all pending calls.
     */
    void cancelAllCalls();

    /**
     * Checks whether there's currently a pending call addressed to the specified node ID.
     */
    bool hasPendingCallToServer(NodeID server_node_id) const;

    /**
     * This method allows to traverse pending calls. If the index is out of range, an invalid call ID will be returned.
     * Warning: average complexity is O(index); worst case complexity is O(size).
     */
    ServiceCallID getCallIDByIndex(unsigned index) const;

    /**
     * Service response callback must be set prior service call.
     */
    const Callback& getCallback() const { return callback_; }
    void setCallback(const Callback& cb) { callback_ = cb; }

    /**
     * Complexity is O(N) of number of pending calls.
     * Note that the number of pending calls will not be updated until the callback is executed.
     */
    unsigned getNumPendingCalls() const { return call_registry_.getSize(); }

    /**
     * Complexity is O(1).
     * Note that the number of pending calls will not be updated until the callback is executed.
     */
    bool hasPendingCalls() const { return !call_registry_.isEmpty(); }

    /**
     * Returns the number of failed attempts to decode received response. Generally, a failed attempt means either:
     * - Transient failure in the transport layer.
     * - Incompatible data types.
     */
    uint32_t getResponseFailureCount() const { return SubscriberType::getFailureCount(); }

    /**
     * Request timeouts. Note that changing the request timeout will not affect calls that are already pending.
     * There is no such config as TX timeout - TX timeouts are configured automagically according to request timeouts.
     * Not recommended to change.
     */
    MonotonicDuration getRequestTimeout() const { return request_timeout_; }
    void setRequestTimeout(MonotonicDuration timeout)
    {
        timeout = max(timeout, getMinRequestTimeout());
        timeout = min(timeout, getMaxRequestTimeout());

        publisher_.setTxTimeout(timeout);
        request_timeout_ = max(timeout, publisher_.getTxTimeout());  // No less than TX timeout
    }

    /**
     * Priority of outgoing request transfers.
     * The remote server is supposed to use the same priority for the response, but it's not guaranteed by
     * the specification.
     */
    TransferPriority getPriority() const { return publisher_.getPriority(); }
    void setPriority(const TransferPriority prio) { publisher_.setPriority(prio); }
};

// ----------------------------------------------------------------------------

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::invokeCallback(ServiceCallResultType& result)
{
    if (coerceOrFallback<bool>(callback_, true))
    {
        callback_(result);
    }
    else
    {
        handleFatalError("Srv client clbk");
    }
}

template <typename DataType_, typename Callback_>
bool ServiceClient<DataType_, Callback_>::shouldAcceptFrame(const RxFrame& frame) const
{
    UAVCAN_ASSERT(frame.getTransferType() == TransferTypeServiceResponse); // Other types filtered out by dispatcher

    return UAVCAN_NULLPTR != call_registry_.find(CallStateMatchingPredicate(ServiceCallID(frame.getSrcNodeID(),
                                                                                          frame.getTransferID())));

}

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::handleReceivedDataStruct(ReceivedDataStructure<ResponseType>& response)
{
    UAVCAN_ASSERT(response.getTransferType() == TransferTypeServiceResponse);

    ServiceCallID call_id(response.getSrcNodeID(), response.getTransferID());
    cancelCall(call_id);
    ServiceCallResultType result(ServiceCallResultType::Success, call_id, response);    // Mutable!
    invokeCallback(result);
}


template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::handleDeadline(MonotonicTime)
{
    UAVCAN_TRACE("ServiceClient", "Shared deadline event received");
    /*
     * Invoking callbacks for timed out call state objects.
     */
    TimeoutCallbackCaller callback_caller(*this);
    call_registry_.template forEach<TimeoutCallbackCaller&>(callback_caller);
    /*
     * Removing timed out objects.
     * This operation cannot be merged with the previous one because that will not work with recursive calls.
     */
    call_registry_.removeAllWhere(&CallState::hasTimedOutPredicate);
    /*
     * Subscriber does not need to be registered if we don't have any pending calls.
     * Removing it makes processing of incoming frames a bit faster.
     */
    if (call_registry_.isEmpty())
    {
        SubscriberType::stop();
    }
}

template <typename DataType_, typename Callback_>
int ServiceClient<DataType_, Callback_>::addCallState(ServiceCallID call_id)
{
    if (call_registry_.isEmpty())
    {
        const int subscriber_res = SubscriberType::startAsServiceResponseListener();
        if (subscriber_res < 0)
        {
            UAVCAN_TRACE("ServiceClient", "Failed to start the subscriber, error: %i", subscriber_res);
            return subscriber_res;
        }
    }

    if (UAVCAN_NULLPTR == call_registry_.template emplace<INode&, ServiceClientBase&,
                                                          ServiceCallID>(SubscriberType::getNode(), *this, call_id))
    {
        SubscriberType::stop();
        return -ErrMemory;
    }

    return 0;
}

template <typename DataType_, typename Callback_>
int ServiceClient<DataType_, Callback_>::call(NodeID server_node_id, const RequestType& request)
{
   ServiceCallID dummy;
   return call(server_node_id, request, dummy);
}

template <typename DataType_, typename Callback_>
int ServiceClient<DataType_, Callback_>::call(NodeID server_node_id, const RequestType& request,
                                              ServiceCallID& out_call_id)
{
    if (!coerceOrFallback<bool>(callback_, true))
    {
        UAVCAN_TRACE("ServiceClient", "Invalid callback");
        return -ErrInvalidConfiguration;
    }

    /*
     * Common procedures that don't depend on the struct data type
     */
    const int prep_res =
        prepareToCall(SubscriberType::getNode(), DataType::getDataTypeFullName(), server_node_id, out_call_id);
    if (prep_res < 0)
    {
        UAVCAN_TRACE("ServiceClient", "Failed to prepare the call, error: %i", prep_res);
        return prep_res;
    }

    /*
     * Initializing the call state - this will start the subscriber ad-hoc
     */
    const int call_state_res = addCallState(out_call_id);
    if (call_state_res < 0)
    {
        UAVCAN_TRACE("ServiceClient", "Failed to add call state, error: %i", call_state_res);
        return call_state_res;
    }

    /*
     * Configuring the listener so it will accept only the matching responses
     * TODO move to init(), but this requires to somewhat refactor GenericSubscriber<> (remove TransferForwarder)
     */
    TransferListenerWithFilter* const tl = SubscriberType::getTransferListener();
    if (tl == UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(0);  // Must have been created
        cancelCall(out_call_id);
        return -ErrLogic;
    }
    tl->installAcceptanceFilter(this);

    /*
     * Publishing the request
     */
    const int publisher_res = publisher_.publish(request, TransferTypeServiceRequest, server_node_id,
                                                 out_call_id.transfer_id);
    if (publisher_res < 0)
    {
        cancelCall(out_call_id);
        return publisher_res;
    }

    UAVCAN_ASSERT(server_node_id == out_call_id.server_node_id);
    return publisher_res;
}

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::cancelCall(ServiceCallID call_id)
{
    call_registry_.removeFirstWhere(CallStateMatchingPredicate(call_id));
    if (call_registry_.isEmpty())
    {
        SubscriberType::stop();
    }
}

template <typename DataType_, typename Callback_>
void ServiceClient<DataType_, Callback_>::cancelAllCalls()
{
    call_registry_.clear();
    SubscriberType::stop();
}

template <typename DataType_, typename Callback_>
bool ServiceClient<DataType_, Callback_>::hasPendingCallToServer(NodeID server_node_id) const
{
    return UAVCAN_NULLPTR != call_registry_.find(ServerSearchPredicate(server_node_id));
}

template <typename DataType_, typename Callback_>
ServiceCallID ServiceClient<DataType_, Callback_>::getCallIDByIndex(unsigned index) const
{
    const CallState* const id = call_registry_.getByIndex(index);
    return (id == UAVCAN_NULLPTR) ? ServiceCallID() : id->getCallID();
}

}

#endif // UAVCAN_NODE_SERVICE_CLIENT_HPP_INCLUDED
